#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>
#include <map>
#include <cmath>
#include <ctime>
#include <chrono>
#include <thread>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <memory>
#include <atomic>
#include <ranges>
#include <sqlite3.h>
#include <cstdlib>
#include <signal.h>

// Configuration constants
constexpr int SCAN_INTERVAL_SECONDS = 60;
constexpr int GRID_INTERVAL_METERS = 5;
constexpr int SIGNAL_THRESHOLD_DBM = -70;
constexpr int ROLLING_PERIOD_DAYS = 30;
constexpr double SIGNAL_PROPAGATION_FACTOR = 2.0;

struct WiFiNetwork {
    std::string ssid;
    std::string bssid;
    int signal_dbm;
    int channel;
    double frequency;
    std::chrono::system_clock::time_point timestamp;
    
    WiFiNetwork() = default;
    WiFiNetwork(const WiFiNetwork&) = default;
    WiFiNetwork& operator=(const WiFiNetwork&) = default;
    WiFiNetwork(WiFiNetwork&&) = default;
    WiFiNetwork& operator=(WiFiNetwork&&) = default;
    ~WiFiNetwork() = default;
};

struct GridPoint {
    int x;
    int y;
    double avg_signal;
    int measurement_count;
    
    GridPoint() = default;
    GridPoint(const GridPoint&) = default;
    GridPoint& operator=(const GridPoint&) = default;
    GridPoint(GridPoint&&) = default;
    GridPoint& operator=(GridPoint&&) = default;
    ~GridPoint() = default;
};

struct RouterRecommendation {
    int x;
    int y;
    double coverage_score;
    double gap_reduction_score;
    
    RouterRecommendation() = default;
    RouterRecommendation(const RouterRecommendation&) = default;
    RouterRecommendation& operator=(const RouterRecommendation&) = default;
    RouterRecommendation(RouterRecommendation&&) = default;
    RouterRecommendation& operator=(RouterRecommendation&&) = default;
    ~RouterRecommendation() = default;
};

class SignalManager {
private:
    static std::atomic<bool> should_continue;
    static SignalManager* instance;

public:
    SignalManager() {
        instance = this;
        should_continue.store(true);
        signal(SIGINT, SignalManager::signalHandler);
        signal(SIGTERM, SignalManager::signalHandler);
    }

    SignalManager(const SignalManager&) = delete;
    SignalManager& operator=(const SignalManager&) = delete;
    
    SignalManager(SignalManager&& other) noexcept {
        if (instance == &other) {
            instance = this;
        }
    }
    
    SignalManager& operator=(SignalManager&& other) noexcept {
        if (this != &other && instance == &other) {
            instance = this;
        }
        return *this;
    }

    ~SignalManager() {
        if (instance == this) {
            instance = nullptr;
        }
    }

    static void signalHandler(int /* signum */) {
        std::cout << "\nShutdown signal received. Cleaning up..." << std::endl;
        should_continue.store(false);
    }

    bool shouldContinue() const {
        return should_continue.load();
    }

    void requestShutdown() {
        should_continue.store(false);
    }
};

std::atomic<bool> SignalManager::should_continue{true};
SignalManager* SignalManager::instance = nullptr;

class WiFiSignalMapper {
private:
    sqlite3* db = nullptr;
    std::string db_path;
    std::map<std::pair<int, int>, GridPoint> signal_grid;
    std::unique_ptr<SignalManager> signal_manager;
    
    void processNetworkLine(std::string_view line, WiFiNetwork& current_network, bool& in_cell, std::vector<WiFiNetwork>& networks) const {
        if (line.contains("Cell")) {
            if (in_cell && !current_network.bssid.empty()) {
                current_network.timestamp = std::chrono::system_clock::now();
                networks.push_back(current_network);
            }
            in_cell = true;
            current_network = WiFiNetwork();
            extractBSSID(line, current_network);
            return;
        }
        
        if (line.contains("ESSID:")) {
            extractSSID(line, current_network);
        } else if (line.contains("Signal level=")) {
            extractSignalStrength(line, current_network);
        } else if (line.contains("Channel:")) {
            extractChannel(line, current_network);
        } else if (line.contains("Frequency:")) {
            extractFrequency(line, current_network);
        }
    }
    
    void extractBSSID(std::string_view line, WiFiNetwork& network) const {
        if (auto pos = line.find("Address: "); pos != std::string::npos) {
            network.bssid = line.substr(pos + 9, 17);
        }
    }
    
    void extractSSID(std::string_view line, WiFiNetwork& network) const {
        auto start = line.find("\"") + 1;
        auto end = line.rfind("\"");
        if (start < end) {
            network.ssid = line.substr(start, end - start);
        }
    }
    
    void extractSignalStrength(std::string_view line, WiFiNetwork& network) const {
        auto pos = line.find("Signal level=");
        std::string signal_str{line.substr(pos + 13)};
        network.signal_dbm = std::stoi(signal_str);
    }
    
    void extractChannel(std::string_view line, WiFiNetwork& network) const {
        auto pos = line.find("Channel:");
        std::string channel_str{line.substr(pos + 8)};
        network.channel = std::stoi(channel_str);
    }
    
    void extractFrequency(std::string_view line, WiFiNetwork& network) const {
        auto pos = line.find("Frequency:");
        std::string freq_str{line.substr(pos + 10)};
        network.frequency = std::stod(freq_str);
    }
    
    bool isGapLocation(int x, int y) const {
        double estimated_signal = estimateSignalAtPoint(x, y);
        return estimated_signal < SIGNAL_THRESHOLD_DBM;
    }
    
    double calculateGapReductionScore(int x, int y, const std::vector<std::pair<int, int>>& gaps) const {
        auto gap_reduction_score = 0.0;
        
        for (const auto& [gap_x, gap_y] : gaps) {
            auto distance = std::sqrt(std::pow(x - gap_x, 2) + std::pow(y - gap_y, 2));
            auto signal_at_gap = -40.0 - 20.0 * SIGNAL_PROPAGATION_FACTOR * std::log10(std::max(1.0, distance));
            
            if (signal_at_gap > SIGNAL_THRESHOLD_DBM) {
                gap_reduction_score += 1.0;
            }
        }
        
        return gap_reduction_score;
    }
    
public:
    explicit WiFiSignalMapper(const std::string& database_path = "wifi_signal_map.db") 
        : db_path(database_path) {
        signal_manager = std::make_unique<SignalManager>();
        initializeDatabase();
    }
    
    WiFiSignalMapper(const WiFiSignalMapper&) = delete;
    WiFiSignalMapper& operator=(const WiFiSignalMapper&) = delete;
    
    WiFiSignalMapper(WiFiSignalMapper&& other) noexcept 
        : db(other.db), db_path(std::move(other.db_path)), 
          signal_grid(std::move(other.signal_grid)),
          signal_manager(std::move(other.signal_manager)) {
        other.db = nullptr;
    }
    
    WiFiSignalMapper& operator=(WiFiSignalMapper&& other) noexcept {
        if (this != &other) {
            if (db) {
                sqlite3_close(db);
            }
            db = other.db;
            db_path = std::move(other.db_path);
            signal_grid = std::move(other.signal_grid);
            signal_manager = std::move(other.signal_manager);
            other.db = nullptr;
        }
        return *this;
    }
    
    ~WiFiSignalMapper() {
        if (db) {
            sqlite3_close(db);
        }
    }
    
    void initializeDatabase() {
        if (auto rc = sqlite3_open(db_path.c_str(), &db); rc) {
            std::cerr << "Can't open database: " << sqlite3_errmsg(db) << std::endl;
            exit(1);
        }
        
        constexpr const char* create_scans_table = R"(
            CREATE TABLE IF NOT EXISTS wifi_scans (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp INTEGER NOT NULL,
                ssid TEXT NOT NULL,
                bssid TEXT NOT NULL,
                signal_dbm INTEGER NOT NULL,
                channel INTEGER,
                frequency REAL,
                grid_x INTEGER,
                grid_y INTEGER
            );
        )";
        
        constexpr const char* create_alerts_table = R"(
            CREATE TABLE IF NOT EXISTS signal_alerts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp INTEGER NOT NULL,
                ssid TEXT NOT NULL,
                bssid TEXT NOT NULL,
                signal_dbm INTEGER NOT NULL,
                alert_type TEXT NOT NULL
            );
        )";
        
        constexpr const char* create_recommendations_table = R"(
            CREATE TABLE IF NOT EXISTS router_recommendations (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp INTEGER NOT NULL,
                grid_x INTEGER NOT NULL,
                grid_y INTEGER NOT NULL,
                coverage_score REAL NOT NULL,
                gap_reduction_score REAL NOT NULL
            );
        )";
        
        char* err_msg = nullptr;
        auto rc_result = sqlite3_exec(db, create_scans_table, nullptr, nullptr, &err_msg);
        if (rc_result != SQLITE_OK) {
            std::cerr << "SQL error: " << err_msg << std::endl;
            sqlite3_free(err_msg);
        }
        
        rc_result = sqlite3_exec(db, create_alerts_table, nullptr, nullptr, &err_msg);
        if (rc_result != SQLITE_OK) {
            std::cerr << "SQL error: " << err_msg << std::endl;
            sqlite3_free(err_msg);
        }
        
        rc_result = sqlite3_exec(db, create_recommendations_table, nullptr, nullptr, &err_msg);
        if (rc_result != SQLITE_OK) {
            std::cerr << "SQL error: " << err_msg << std::endl;
            sqlite3_free(err_msg);
        }
        
        sqlite3_exec(db, "CREATE INDEX IF NOT EXISTS idx_timestamp ON wifi_scans(timestamp);", nullptr, nullptr, nullptr);
        sqlite3_exec(db, "CREATE INDEX IF NOT EXISTS idx_grid ON wifi_scans(grid_x, grid_y);", nullptr, nullptr, nullptr);
    }
    
    std::vector<WiFiNetwork> scanWiFiNetworks() const {
        std::vector<WiFiNetwork> networks;
        
        FILE* pipe = popen("sudo iwlist wlan0 scan 2>/dev/null", "r");
        if (!pipe) {
            std::cerr << "Failed to execute iwlist command" << std::endl;
            return networks;
        }
        
        std::string buffer;
        buffer.resize(256);
        WiFiNetwork current_network;
        bool in_cell = false;
        
        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
            std::string line(buffer.data());
            processNetworkLine(line, current_network, in_cell, networks);
        }
        
        if (in_cell && !current_network.bssid.empty()) {
            current_network.timestamp = std::chrono::system_clock::now();
            networks.push_back(current_network);
        }
        
        pclose(pipe);
        return networks;
    }
    
    void storeScanResults(const std::vector<WiFiNetwork>& networks, int grid_x = 0, int grid_y = 0) {
        constexpr const char* insert_sql = R"(
            INSERT INTO wifi_scans (timestamp, ssid, bssid, signal_dbm, channel, frequency, grid_x, grid_y)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?);
        )";
        
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, insert_sql, -1, &stmt, nullptr);
        
        for (const auto& network : networks) {
            auto time_t_timestamp = std::chrono::system_clock::to_time_t(network.timestamp);
            sqlite3_bind_int64(stmt, 1, time_t_timestamp);
            sqlite3_bind_text(stmt, 2, network.ssid.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 3, network.bssid.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_int(stmt, 4, network.signal_dbm);
            sqlite3_bind_int(stmt, 5, network.channel);
            sqlite3_bind_double(stmt, 6, network.frequency);
            sqlite3_bind_int(stmt, 7, grid_x);
            sqlite3_bind_int(stmt, 8, grid_y);
            
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
            
            if (network.signal_dbm < SIGNAL_THRESHOLD_DBM) {
                generateAlert(network, "LOW_SIGNAL");
            }
        }
        
        sqlite3_finalize(stmt);
    }
    
    void generateAlert(const WiFiNetwork& network, std::string_view alert_type) const {
        std::cout << "\n[ALERT] " << alert_type << " - " 
                  << "SSID: " << network.ssid 
                  << ", Signal: " << network.signal_dbm << " dBm" 
                  << " (Below threshold: " << SIGNAL_THRESHOLD_DBM << " dBm)" << std::endl;
        
        constexpr const char* insert_alert = R"(
            INSERT INTO signal_alerts (timestamp, ssid, bssid, signal_dbm, alert_type)
            VALUES (?, ?, ?, ?, ?);
        )";
        
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, insert_alert, -1, &stmt, nullptr);
        
        auto time_t_timestamp = std::chrono::system_clock::to_time_t(network.timestamp);
        sqlite3_bind_int64(stmt, 1, time_t_timestamp);
        sqlite3_bind_text(stmt, 2, network.ssid.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_text(stmt, 3, network.bssid.c_str(), -1, SQLITE_STATIC);
        sqlite3_bind_int(stmt, 4, network.signal_dbm);
        std::string alert_type_str{alert_type};
        sqlite3_bind_text(stmt, 5, alert_type_str.c_str(), -1, SQLITE_STATIC);
        
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
    }
    
    void calculateSignalHeatmap() {
        signal_grid.clear();
        
        auto cutoff_time = std::chrono::system_clock::now() - std::chrono::hours(ROLLING_PERIOD_DAYS * 24);
        auto cutoff_time_t = std::chrono::system_clock::to_time_t(cutoff_time);
        
        constexpr const char* query = R"(
            SELECT grid_x, grid_y, AVG(signal_dbm) as avg_signal, COUNT(*) as count
            FROM wifi_scans
            WHERE timestamp > ?
            GROUP BY grid_x, grid_y;
        )";
        
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, query, -1, &stmt, nullptr);
        sqlite3_bind_int64(stmt, 1, cutoff_time_t);
        
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            auto x = sqlite3_column_int(stmt, 0);
            auto y = sqlite3_column_int(stmt, 1);
            auto avg_signal = sqlite3_column_double(stmt, 2);
            auto count = sqlite3_column_int(stmt, 3);
            
            GridPoint point = {x, y, avg_signal, count};
            signal_grid[{x, y}] = point;
        }
        
        sqlite3_finalize(stmt);
    }
    
    double estimateSignalAtPoint(int x, int y) const {
        auto weighted_sum = 0.0;
        auto weight_total = 0.0;
        
        for (const auto& [coord, point] : signal_grid) {
            auto distance = std::sqrt(std::pow(x - coord.first, 2) + std::pow(y - coord.second, 2));
            if (distance < 0.1) {
                return point.avg_signal;
            }
            
            auto weight = 1.0 / std::pow(distance, SIGNAL_PROPAGATION_FACTOR);
            weighted_sum += point.avg_signal * weight;
            weight_total += weight;
        }
        
        return weight_total > 0 ? weighted_sum / weight_total : -100.0;
    }
    
    std::vector<std::pair<int, int>> findCoverageGaps(int min_x, int max_x, int min_y, int max_y) const {
        std::vector<std::pair<int, int>> gaps;
        
        for (int x = min_x; x <= max_x; x += GRID_INTERVAL_METERS) {
            for (int y = min_y; y <= max_y; y += GRID_INTERVAL_METERS) {
                if (isGapLocation(x, y)) {
                    gaps.push_back({x, y});
                }
            }
        }
        
        return gaps;
    }
    
    std::vector<RouterRecommendation> calculateOptimalRouterPlacement(int min_x, int max_x, int min_y, int max_y) const {
        std::vector<RouterRecommendation> recommendations;
        auto gaps = findCoverageGaps(min_x, max_x, min_y, max_y);
        
        for (int x = min_x; x <= max_x; x += GRID_INTERVAL_METERS) {
            for (int y = min_y; y <= max_y; y += GRID_INTERVAL_METERS) {
                auto coverage_score = 0.0;
                auto gap_reduction_score = calculateGapReductionScore(x, y, gaps);
                
                auto existing_signal = estimateSignalAtPoint(x, y);
                coverage_score = 100.0 - std::abs(existing_signal);
                
                recommendations.emplace_back(RouterRecommendation{x, y, coverage_score, gap_reduction_score});
            }
        }
        
        std::ranges::sort(recommendations, 
            [](const RouterRecommendation& a, const RouterRecommendation& b) {
                return (a.gap_reduction_score + a.coverage_score) > (b.gap_reduction_score + b.coverage_score);
            });
        
        if (recommendations.size() > 5) {
            recommendations.resize(5);
        }
        
        return recommendations;
    }
    
    void generateTrendAnalysis() const {
        std::cout << "\n=== Historical Trend Analysis (Last 30 Days) ===" << std::endl;
        
        auto cutoff_time = std::chrono::system_clock::now() - std::chrono::hours(ROLLING_PERIOD_DAYS * 24);
        auto cutoff_time_t = std::chrono::system_clock::to_time_t(cutoff_time);
        
        constexpr const char* trend_query = R"(
            SELECT 
                DATE(timestamp, 'unixepoch') as date,
                AVG(signal_dbm) as avg_signal,
                MIN(signal_dbm) as min_signal,
                MAX(signal_dbm) as max_signal,
                COUNT(*) as measurements
            FROM wifi_scans
            WHERE timestamp > ?
            GROUP BY date
            ORDER BY date DESC
            LIMIT 7;
        )";
        
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, trend_query, -1, &stmt, nullptr);
        sqlite3_bind_int64(stmt, 1, cutoff_time_t);
        
        std::cout << "\nDaily Signal Strength Summary:" << std::endl;
        std::cout << "Date       | Avg Signal | Min Signal | Max Signal | Measurements" << std::endl;
        std::cout << "-----------|------------|------------|------------|-------------" << std::endl;
        
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            const char* date = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            auto avg_signal = sqlite3_column_double(stmt, 1);
            auto min_signal = sqlite3_column_int(stmt, 2);
            auto max_signal = sqlite3_column_int(stmt, 3);
            auto count = sqlite3_column_int(stmt, 4);
            
            std::cout << std::setw(10) << date << " | "
                      << std::setw(10) << std::fixed << std::setprecision(1) << avg_signal << " | "
                      << std::setw(10) << min_signal << " | "
                      << std::setw(10) << max_signal << " | "
                      << std::setw(12) << count << std::endl;
        }
        
        sqlite3_finalize(stmt);
        
        constexpr const char* network_query = R"(
            SELECT 
                ssid,
                AVG(signal_dbm) as avg_signal,
                COUNT(*) as measurements,
                COUNT(DISTINCT DATE(timestamp, 'unixepoch')) as days_seen
            FROM wifi_scans
            WHERE timestamp > ?
            GROUP BY ssid
            ORDER BY avg_signal DESC;
        )";
        
        sqlite3_prepare_v2(db, network_query, -1, &stmt, nullptr);
        sqlite3_bind_int64(stmt, 1, cutoff_time_t);
        
        std::cout << "\nNetwork Performance Summary:" << std::endl;
        std::cout << "SSID                          | Avg Signal | Measurements | Days Seen" << std::endl;
        std::cout << "------------------------------|------------|--------------|----------" << std::endl;
        
        while (sqlite3_step(stmt) == SQLITE_ROW) {
            const char* ssid = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            auto avg_signal = sqlite3_column_double(stmt, 1);
            auto measurements = sqlite3_column_int(stmt, 2);
            auto days_seen = sqlite3_column_int(stmt, 3);
            
            std::cout << std::left << std::setw(29) << ssid << " | "
                      << std::right << std::setw(10) << std::fixed << std::setprecision(1) << avg_signal << " | "
                      << std::setw(12) << measurements << " | "
                      << std::setw(9) << days_seen << std::endl;
        }
        
        sqlite3_finalize(stmt);
    }
    
    void generateInsights() {
        std::cout << "\n=== Actionable Infrastructure Insights ===" << std::endl;
        
        calculateSignalHeatmap();
        
        constexpr int min_x = -100;
        constexpr int max_x = 100;
        constexpr int min_y = -100;
        constexpr int max_y = 100;
        
        auto gaps = findCoverageGaps(min_x, max_x, min_y, max_y);
        std::cout << "\nCoverage Gaps Detected: " << gaps.size() << std::endl;
        
        if (!gaps.empty()) {
            std::cout << "Gap Locations (first 10):" << std::endl;
            for (size_t i = 0; i < std::min(gaps.size(), size_t(10)); ++i) {
                std::cout << "  - Grid position (" << gaps[i].first << ", " << gaps[i].second << ")" << std::endl;
            }
        }
        
        auto recommendations = calculateOptimalRouterPlacement(min_x, max_x, min_y, max_y);
        
        std::cout << "\nOptimal Router Placement Recommendations:" << std::endl;
        std::cout << "Position      | Coverage Score | Gap Reduction | Combined Score" << std::endl;
        std::cout << "--------------|----------------|---------------|---------------" << std::endl;
        
        for (const auto& rec : recommendations) {
            auto combined_score = rec.coverage_score + rec.gap_reduction_score;
            std::cout << "(" << std::setw(4) << rec.x << ", " << std::setw(4) << rec.y << ") | "
                      << std::setw(14) << std::fixed << std::setprecision(1) << rec.coverage_score << " | "
                      << std::setw(13) << std::fixed << std::setprecision(1) << rec.gap_reduction_score << " | "
                      << std::setw(14) << std::fixed << std::setprecision(1) << combined_score << std::endl;
        }
        
        constexpr const char* insert_rec = R"(
            INSERT INTO router_recommendations (timestamp, grid_x, grid_y, coverage_score, gap_reduction_score)
            VALUES (?, ?, ?, ?, ?);
        )";
        
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, insert_rec, -1, &stmt, nullptr);
        
        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        
        for (const auto& rec : recommendations) {
            sqlite3_bind_int64(stmt, 1, now_time_t);
            sqlite3_bind_int(stmt, 2, rec.x);
            sqlite3_bind_int(stmt, 3, rec.y);
            sqlite3_bind_double(stmt, 4, rec.coverage_score);
            sqlite3_bind_double(stmt, 5, rec.gap_reduction_score);
            
            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }
        
        sqlite3_finalize(stmt);
        
        constexpr const char* alerts_query = R"(
            SELECT COUNT(*) as alert_count
            FROM signal_alerts
            WHERE timestamp > ?;
        )";
        
        sqlite3_prepare_v2(db, alerts_query, -1, &stmt, nullptr);
        auto yesterday = now_time_t - (24 * 60 * 60);
        sqlite3_bind_int64(stmt, 1, yesterday);
        
        if (sqlite3_step(stmt) == SQLITE_ROW) {
            auto alert_count = sqlite3_column_int(stmt, 0);
            std::cout << "\nSignal Alerts (Last 24 hours): " << alert_count << std::endl;
            
            if (alert_count > 10) {
                std::cout << "WARNING: High number of signal alerts detected. Consider immediate infrastructure review." << std::endl;
            }
        }
        
        sqlite3_finalize(stmt);
    }
    
    void startMonitoring() {
        std::cout << "WiFi Signal Mapping System Started" << std::endl;
        std::cout << "Scanning every " << SCAN_INTERVAL_SECONDS << " seconds..." << std::endl;
        std::cout << "Press Ctrl+C to stop" << std::endl;
        
        int scan_count = 0;
        
        while (signal_manager->shouldContinue()) {
            auto start_time = std::chrono::steady_clock::now();
            
            ++scan_count;
            
            auto current_time = std::chrono::system_clock::now();
            auto current_time_t = std::chrono::system_clock::to_time_t(current_time);
            
            std::cout << "\n[" << current_time_t << "] Performing WiFi scan #" << scan_count << std::endl;
            
            if (auto networks = scanWiFiNetworks(); !networks.empty()) {
                std::cout << "Found " << networks.size() << " networks" << std::endl;
                
                storeScanResults(networks, 0, 0);
                
                if (scan_count % 10 == 0) {
                    generateTrendAnalysis();
                    generateInsights();
                }
            } else {
                std::cout << "No networks found or scan failed" << std::endl;
            }
            
            auto end_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
            auto sleep_time = std::max(1, SCAN_INTERVAL_SECONDS - static_cast<int>(elapsed));
            
            for (int i = 0; i < sleep_time && signal_manager->shouldContinue(); ++i) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        
        std::cout << "\nMonitoring stopped. Total scans: " << scan_count << std::endl;
    }
    
    void performMaintenance() {
        auto cutoff_time = std::chrono::system_clock::now() - std::chrono::hours(ROLLING_PERIOD_DAYS * 24 * 2);
        auto cutoff_time_t = std::chrono::system_clock::to_time_t(cutoff_time);
        
        constexpr const char* cleanup_sql = "DELETE FROM wifi_scans WHERE timestamp < ?;";
        sqlite3_stmt* stmt;
        sqlite3_prepare_v2(db, cleanup_sql, -1, &stmt, nullptr);
        sqlite3_bind_int64(stmt, 1, cutoff_time_t);
        sqlite3_step(stmt);
        sqlite3_finalize(stmt);
        
        sqlite3_exec(db, "VACUUM;", nullptr, nullptr, nullptr);
    }
};

int main() {
    try {
        auto mapper = std::make_unique<WiFiSignalMapper>("wifi_signal_map.db");
        
        mapper->startMonitoring();
        
        mapper->performMaintenance();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
