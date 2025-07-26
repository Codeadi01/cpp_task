#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <chrono>
#include <thread>
#include <regex>
#include <cmath>
#include <algorithm>
#include <memory>
#include <mutex>
#include <queue>
#include <sqlite3.h>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <signal.h>

class WiFiSignalMapper {
private:
    struct NetworkData {
        std::string ssid;
        std::string bssid;
        int signal_strength_dbm;
        double frequency;
        std::chrono::system_clock::time_point timestamp;
        double x_position;
        double y_position;
    };

    struct GridCell {
        double x, y;
        std::vector<int> signal_values;
        double avg_signal = -100.0;
        int measurement_count = 0;
    };

    struct RouterRecommendation {
        double x, y;
        double coverage_score;
        std::string reasoning;
    };

    sqlite3* db;
    std::mutex db_mutex;
    std::mutex data_mutex;
    bool running;
    const int SIGNAL_THRESHOLD = -70;
    const int GRID_INTERVAL = 5; // 5 meters
    const int SCAN_INTERVAL = 60; // 60 seconds
    const int HISTORY_DAYS = 30;
    
    // Current position (can be configured or GPS-based)
    double current_x = 0.0;
    double current_y = 0.0;
    
    std::vector<NetworkData> recent_scans;
    std::unordered_map<std::string, std::queue<NetworkData>> network_history;

public:
    WiFiSignalMapper() : db(nullptr), running(false) {
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
    }

    ~WiFiSignalMapper() {
        cleanup();
    }

    static void signalHandler(int signal) {
        std::cout << "\nShutting down WiFi Signal Mapper..." << std::endl;
        exit(0);
    }

    bool initialize() {
        if (!initializeDatabase()) {
            std::cerr << "Failed to initialize database" << std::endl;
            return false;
        }
        
        std::cout << "WiFi Signal Mapper initialized successfully" << std::endl;
        return true;
    }

    void cleanup() {
        running = false;
        if (db) {
            sqlite3_close(db);
            db = nullptr;
        }
    }

private:
    bool initializeDatabase() {
        int rc = sqlite3_open("wifi_signal_data.db", &db);
        if (rc != SQLITE_OK) {
            std::cerr << "Cannot open database: " << sqlite3_errmsg(db) << std::endl;
            return false;
        }

        const char* create_tables_sql = R"(
            CREATE TABLE IF NOT EXISTS signal_measurements (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ssid TEXT NOT NULL,
                bssid TEXT NOT NULL,
                signal_strength_dbm INTEGER NOT NULL,
                frequency REAL NOT NULL,
                x_position REAL NOT NULL,
                y_position REAL NOT NULL,
                timestamp INTEGER NOT NULL
            );

            CREATE TABLE IF NOT EXISTS alerts (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ssid TEXT NOT NULL,
                bssid TEXT NOT NULL,
                signal_strength_dbm INTEGER NOT NULL,
                x_position REAL NOT NULL,
                y_position REAL NOT NULL,
                alert_type TEXT NOT NULL,
                timestamp INTEGER NOT NULL
            );

            CREATE TABLE IF NOT EXISTS coverage_analysis (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                grid_x REAL NOT NULL,
                grid_y REAL NOT NULL,
                avg_signal_strength REAL NOT NULL,
                measurement_count INTEGER NOT NULL,
                coverage_quality TEXT NOT NULL,
                timestamp INTEGER NOT NULL
            );

            CREATE INDEX IF NOT EXISTS idx_timestamp ON signal_measurements(timestamp);
            CREATE INDEX IF NOT EXISTS idx_bssid ON signal_measurements(bssid);
            CREATE INDEX IF NOT EXISTS idx_position ON signal_measurements(x_position, y_position);
        )";

        char* err_msg = nullptr;
        rc = sqlite3_exec(db, create_tables_sql, nullptr, nullptr, &err_msg);
        if (rc != SQLITE_OK) {
            std::cerr << "SQL error: " << err_msg << std::endl;
            sqlite3_free(err_msg);
            return false;
        }

        return true;
    }

    std::string executeCommand(const std::string& command) {
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
        
        if (!pipe) {
            return "";
        }

        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe.get()) != nullptr) {
            result += buffer;
        }
        
        return result;
    }

    std::vector<NetworkData> scanWiFiNetworks() {
        std::vector<NetworkData> networks;
        auto now = std::chrono::system_clock::now();
        
        // Execute iwlist scan command
        std::string scan_output = executeCommand("sudo iwlist scan 2>/dev/null");
        
        if (scan_output.empty()) {
            std::cout << "Warning: No WiFi scan data available" << std::endl;
            return networks;
        }

        // Parse iwlist output
        std::regex cell_regex(R"(Cell \d+)");
        std::regex ssid_regex(R"(ESSID:"([^"]*)")");
        std::regex bssid_regex(R"(Address: ([0-9A-Fa-f:]{17}))");
        std::regex signal_regex(R"(Signal level=(-?\d+) dBm)");
        std::regex freq_regex(R"(Frequency:(\d+\.?\d*) GHz)");

        std::istringstream stream(scan_output);
        std::string line;
        NetworkData current_network;
        bool in_cell = false;

        while (std::getline(stream, line)) {
            std::smatch match;
            
            if (std::regex_search(line, match, cell_regex)) {
                if (in_cell && !current_network.ssid.empty()) {
                    current_network.timestamp = now;
                    current_network.x_position = current_x;
                    current_network.y_position = current_y;
                    networks.push_back(current_network);
                }
                current_network = NetworkData();
                in_cell = true;
            } else if (std::regex_search(line, match, bssid_regex)) {
                current_network.bssid = match[1].str();
            } else if (std::regex_search(line, match, ssid_regex)) {
                current_network.ssid = match[1].str();
            } else if (std::regex_search(line, match, signal_regex)) {
                current_network.signal_strength_dbm = std::stoi(match[1].str());
            } else if (std::regex_search(line, match, freq_regex)) {
                current_network.frequency = std::stod(match[1].str());
            }
        }

        // Add the last network
        if (in_cell && !current_network.ssid.empty()) {
            current_network.timestamp = now;
            current_network.x_position = current_x;
            current_network.y_position = current_y;
            networks.push_back(current_network);
        }

        return networks;
    }

    void storeNetworkData(const std::vector<NetworkData>& networks) {
        std::lock_guard<std::mutex> lock(db_mutex);
        
        const char* insert_sql = R"(
            INSERT INTO signal_measurements 
            (ssid, bssid, signal_strength_dbm, frequency, x_position, y_position, timestamp)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        )";

        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db, insert_sql, -1, &stmt, nullptr);
        
        if (rc != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(db) << std::endl;
            return;
        }

        for (const auto& network : networks) {
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                network.timestamp.time_since_epoch()).count();

            sqlite3_bind_text(stmt, 1, network.ssid.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 2, network.bssid.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_int(stmt, 3, network.signal_strength_dbm);
            sqlite3_bind_double(stmt, 4, network.frequency);
            sqlite3_bind_double(stmt, 5, network.x_position);
            sqlite3_bind_double(stmt, 6, network.y_position);
            sqlite3_bind_int64(stmt, 7, timestamp);

            rc = sqlite3_step(stmt);
            if (rc != SQLITE_DONE) {
                std::cerr << "Failed to insert data: " << sqlite3_errmsg(db) << std::endl;
            }
            sqlite3_reset(stmt);
        }

        sqlite3_finalize(stmt);
    }

    void checkSignalAlerts(const std::vector<NetworkData>& networks) {
        for (const auto& network : networks) {
            if (network.signal_strength_dbm < SIGNAL_THRESHOLD) {
                generateAlert(network, "LOW_SIGNAL");
                std::cout << "ALERT: Low signal detected - SSID: " << network.ssid 
                         << ", Signal: " << network.signal_strength_dbm << " dBm" 
                         << ", Position: (" << network.x_position << ", " << network.y_position << ")"
                         << std::endl;
            }
        }
    }

    void generateAlert(const NetworkData& network, const std::string& alert_type) {
        std::lock_guard<std::mutex> lock(db_mutex);
        
        const char* insert_sql = R"(
            INSERT INTO alerts 
            (ssid, bssid, signal_strength_dbm, x_position, y_position, alert_type, timestamp)
            VALUES (?, ?, ?, ?, ?, ?, ?)
        )";

        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db, insert_sql, -1, &stmt, nullptr);
        
        if (rc == SQLITE_OK) {
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
                network.timestamp.time_since_epoch()).count();

            sqlite3_bind_text(stmt, 1, network.ssid.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_text(stmt, 2, network.bssid.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_int(stmt, 3, network.signal_strength_dbm);
            sqlite3_bind_double(stmt, 4, network.x_position);
            sqlite3_bind_double(stmt, 5, network.y_position);
            sqlite3_bind_text(stmt, 6, alert_type.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_int64(stmt, 7, timestamp);

            sqlite3_step(stmt);
            sqlite3_finalize(stmt);
        }
    }

    std::vector<GridCell> generateHeatmapGrid() {
        std::vector<GridCell> grid;
        std::lock_guard<std::mutex> lock(db_mutex);

        // Get data from last 24 hours for grid generation
        auto now = std::chrono::system_clock::now();
        auto day_ago = now - std::chrono::hours(24);
        auto timestamp_cutoff = std::chrono::duration_cast<std::chrono::seconds>(
            day_ago.time_since_epoch()).count();

        const char* query_sql = R"(
            SELECT x_position, y_position, signal_strength_dbm 
            FROM signal_measurements 
            WHERE timestamp > ?
            ORDER BY x_position, y_position
        )";

        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db, query_sql, -1, &stmt, nullptr);
        
        if (rc != SQLITE_OK) {
            return grid;
        }

        sqlite3_bind_int64(stmt, 1, timestamp_cutoff);

        std::map<std::pair<int, int>, GridCell> grid_map;

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            double x = sqlite3_column_double(stmt, 0);
            double y = sqlite3_column_double(stmt, 1);
            int signal = sqlite3_column_int(stmt, 2);

            // Round to nearest grid point
            int grid_x = static_cast<int>(std::round(x / GRID_INTERVAL)) * GRID_INTERVAL;
            int grid_y = static_cast<int>(std::round(y / GRID_INTERVAL)) * GRID_INTERVAL;

            auto grid_key = std::make_pair(grid_x, grid_y);
            
            if (grid_map.find(grid_key) == grid_map.end()) {
                grid_map[grid_key] = GridCell{static_cast<double>(grid_x), static_cast<double>(grid_y)};
            }

            grid_map[grid_key].signal_values.push_back(signal);
            grid_map[grid_key].measurement_count++;
        }

        sqlite3_finalize(stmt);

        // Calculate averages and convert to vector
        for (auto& [key, cell] : grid_map) {
            if (!cell.signal_values.empty()) {
                double sum = 0;
                for (int signal : cell.signal_values) {
                    sum += signal;
                }
                cell.avg_signal = sum / cell.signal_values.size();
                grid.push_back(cell);
            }
        }

        return grid;
    }

    std::vector<RouterRecommendation> recommendRouterPlacements(const std::vector<GridCell>& grid) {
        std::vector<RouterRecommendation> recommendations;
        
        if (grid.empty()) {
            return recommendations;
        }

        // Find coverage gaps (areas with poor signal)
        std::vector<GridCell> weak_areas;
        for (const auto& cell : grid) {
            if (cell.avg_signal < SIGNAL_THRESHOLD) {
                weak_areas.push_back(cell);
            }
        }

        // Group weak areas into clusters for router placement
        std::vector<std::vector<GridCell>> clusters = clusterWeakAreas(weak_areas);

        for (const auto& cluster : clusters) {
            if (cluster.empty()) continue;

            // Calculate centroid of cluster
            double center_x = 0, center_y = 0;
            for (const auto& cell : cluster) {
                center_x += cell.x;
                center_y += cell.y;
            }
            center_x /= cluster.size();
            center_y /= cluster.size();

            // Calculate coverage score based on potential improvement
            double coverage_score = calculateCoverageScore(center_x, center_y, grid);
            
            std::stringstream reasoning;
            reasoning << "Cluster of " << cluster.size() << " weak signal areas. "
                     << "Average signal improvement: " << std::fixed << std::setprecision(1) 
                     << coverage_score << " dBm";

            recommendations.push_back({center_x, center_y, coverage_score, reasoning.str()});
        }

        // Sort by coverage score (higher is better)
        std::sort(recommendations.begin(), recommendations.end(),
            [](const RouterRecommendation& a, const RouterRecommendation& b) {
                return a.coverage_score > b.coverage_score;
            });

        return recommendations;
    }

    std::vector<std::vector<GridCell>> clusterWeakAreas(const std::vector<GridCell>& weak_areas) {
        std::vector<std::vector<GridCell>> clusters;
        std::vector<bool> visited(weak_areas.size(), false);
        const double CLUSTER_DISTANCE = GRID_INTERVAL * 3.0; // 15 meters

        for (size_t i = 0; i < weak_areas.size(); ++i) {
            if (visited[i]) continue;

            std::vector<GridCell> cluster;
            std::queue<size_t> to_visit;
            to_visit.push(i);
            visited[i] = true;

            while (!to_visit.empty()) {
                size_t current = to_visit.front();
                to_visit.pop();
                cluster.push_back(weak_areas[current]);

                for (size_t j = 0; j < weak_areas.size(); ++j) {
                    if (visited[j]) continue;

                    double distance = std::sqrt(
                        std::pow(weak_areas[current].x - weak_areas[j].x, 2) +
                        std::pow(weak_areas[current].y - weak_areas[j].y, 2)
                    );

                    if (distance <= CLUSTER_DISTANCE) {
                        visited[j] = true;
                        to_visit.push(j);
                    }
                }
            }

            if (cluster.size() >= 2) { // Only consider clusters with multiple weak areas
                clusters.push_back(cluster);
            }
        }

        return clusters;
    }

    double calculateCoverageScore(double x, double y, const std::vector<GridCell>& grid) {
        double total_improvement = 0;
        int affected_cells = 0;
        const double ROUTER_RANGE = 50.0; // Effective range in meters

        for (const auto& cell : grid) {
            double distance = std::sqrt(std::pow(cell.x - x, 2) + std::pow(cell.y - y, 2));
            
            if (distance <= ROUTER_RANGE) {
                // Simplified path loss model: Signal = -30 - 20*log10(distance)
                double predicted_signal = -30 - 20 * std::log10(std::max(1.0, distance));
                double improvement = std::max(0.0, predicted_signal - cell.avg_signal);
                total_improvement += improvement;
                affected_cells++;
            }
        }

        return affected_cells > 0 ? total_improvement / affected_cells : 0;
    }

    void performHistoricalAnalysis() {
        std::lock_guard<std::mutex> lock(db_mutex);
        
        auto now = std::chrono::system_clock::now();
        auto month_ago = now - std::chrono::hours(24 * HISTORY_DAYS);
        auto timestamp_cutoff = std::chrono::duration_cast<std::chrono::seconds>(
            month_ago.time_since_epoch()).count();

        const char* trend_sql = R"(
            SELECT ssid, bssid, 
                   AVG(signal_strength_dbm) as avg_signal,
                   MIN(signal_strength_dbm) as min_signal,
                   MAX(signal_strength_dbm) as max_signal,
                   COUNT(*) as measurement_count,
                   (julianday('now') - julianday(datetime(timestamp, 'unixepoch'))) as days_ago
            FROM signal_measurements 
            WHERE timestamp > ?
            GROUP BY ssid, bssid
            HAVING COUNT(*) > 10
            ORDER BY avg_signal DESC
        )";

        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db, trend_sql, -1, &stmt, nullptr);
        
        if (rc != SQLITE_OK) {
            return;
        }

        sqlite3_bind_int64(stmt, 1, timestamp_cutoff);

        std::cout << "\n=== Historical Analysis (Last " << HISTORY_DAYS << " Days) ===" << std::endl;
        std::cout << "Network Performance Summary:" << std::endl;
        std::cout << "SSID\t\t\tAvg Signal\tMin/Max\t\tMeasurements" << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            std::string ssid = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));
            double avg_signal = sqlite3_column_double(stmt, 2);
            int min_signal = sqlite3_column_int(stmt, 3);
            int max_signal = sqlite3_column_int(stmt, 4);
            int count = sqlite3_column_int(stmt, 5);

            std::cout << ssid.substr(0, 15) << "\t\t" 
                     << std::fixed << std::setprecision(1) << avg_signal << " dBm\t\t"
                     << min_signal << "/" << max_signal << " dBm\t\t"
                     << count << std::endl;
        }

        sqlite3_finalize(stmt);
    }

    void storeCoverageAnalysis(const std::vector<GridCell>& grid) {
        std::lock_guard<std::mutex> lock(db_mutex);
        
        const char* insert_sql = R"(
            INSERT INTO coverage_analysis 
            (grid_x, grid_y, avg_signal_strength, measurement_count, coverage_quality, timestamp)
            VALUES (?, ?, ?, ?, ?, ?)
        )";

        sqlite3_stmt* stmt;
        int rc = sqlite3_prepare_v2(db, insert_sql, -1, &stmt, nullptr);
        
        if (rc != SQLITE_OK) {
            return;
        }

        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            now.time_since_epoch()).count();

        for (const auto& cell : grid) {
            std::string quality;
            if (cell.avg_signal >= -50) quality = "Excellent";
            else if (cell.avg_signal >= -60) quality = "Good";
            else if (cell.avg_signal >= -70) quality = "Fair";
            else quality = "Poor";

            sqlite3_bind_double(stmt, 1, cell.x);
            sqlite3_bind_double(stmt, 2, cell.y);
            sqlite3_bind_double(stmt, 3, cell.avg_signal);
            sqlite3_bind_int(stmt, 4, cell.measurement_count);
            sqlite3_bind_text(stmt, 5, quality.c_str(), -1, SQLITE_STATIC);
            sqlite3_bind_int64(stmt, 6, timestamp);

            sqlite3_step(stmt);
            sqlite3_reset(stmt);
        }

        sqlite3_finalize(stmt);
    }

    void printOptimizationReport(const std::vector<GridCell>& grid, 
                               const std::vector<RouterRecommendation>& recommendations) {
        std::cout << "\n=== WiFi Infrastructure Optimization Report ===" << std::endl;
        
        // Coverage Statistics
        int excellent = 0, good = 0, fair = 0, poor = 0;
        double total_coverage_area = 0;
        
        for (const auto& cell : grid) {
            total_coverage_area += GRID_INTERVAL * GRID_INTERVAL;
            if (cell.avg_signal >= -50) excellent++;
            else if (cell.avg_signal >= -60) good++;
            else if (cell.avg_signal >= -70) fair++;
            else poor++;
        }

        std::cout << "\nCoverage Quality Distribution:" << std::endl;
        std::cout << "Excellent (>= -50 dBm): " << excellent << " areas (" 
                 << std::fixed << std::setprecision(1) << (excellent * 100.0 / grid.size()) << "%)" << std::endl;
        std::cout << "Good (-50 to -60 dBm): " << good << " areas (" 
                 << std::fixed << std::setprecision(1) << (good * 100.0 / grid.size()) << "%)" << std::endl;
        std::cout << "Fair (-60 to -70 dBm): " << fair << " areas (" 
                 << std::fixed << std::setprecision(1) << (fair * 100.0 / grid.size()) << "%)" << std::endl;
        std::cout << "Poor (< -70 dBm): " << poor << " areas (" 
                 << std::fixed << std::setprecision(1) << (poor * 100.0 / grid.size()) << "%)" << std::endl;

        // Router Recommendations
        if (!recommendations.empty()) {
            std::cout << "\nRouter Placement Recommendations:" << std::endl;
            for (size_t i = 0; i < std::min(recommendations.size(), size_t(5)); ++i) {
                const auto& rec = recommendations[i];
                std::cout << (i + 1) << ". Position: (" << std::fixed << std::setprecision(1) 
                         << rec.x << ", " << rec.y << ") - Score: " << rec.coverage_score 
                         << " - " << rec.reasoning << std::endl;
            }
        } else {
            std::cout << "\nNo critical router placement needs identified." << std::endl;
        }

        // Performance Metrics
        std::cout << "\nCurrent Monitoring Status:" << std::endl;
        std::cout << "Grid Resolution: " << GRID_INTERVAL << "m intervals" << std::endl;
        std::cout << "Total Monitored Area: " << std::fixed << std::setprecision(0) 
                 << total_coverage_area << " m²" << std::endl;
        std::cout << "Signal Threshold: " << SIGNAL_THRESHOLD << " dBm" << std::endl;
        std::cout << "Scan Interval: " << SCAN_INTERVAL << " seconds" << std::endl;
    }

public:
    void run() {
        if (!initialize()) {
            return;
        }

        running = true;
        std::cout << "Starting WiFi Signal Mapper - Continuous Monitoring Mode" << std::endl;
        std::cout << "Press Ctrl+C to stop..." << std::endl;

        int cycle_count = 0;
        const int ANALYSIS_CYCLE = 10; // Perform full analysis every 10 cycles (10 minutes)

        while (running) {
            try {
                // Scan for WiFi networks
                auto networks = scanWiFiNetworks();
                
                if (!networks.empty()) {
                    // Store data
                    storeNetworkData(networks);
                    
                    // Check for alerts
                    checkSignalAlerts(networks);
                    
                    // Update recent scans
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        recent_scans = networks;
                    }

                    std::cout << "Scan completed: " << networks.size() << " networks detected" << std::endl;
                    
                    // Perform comprehensive analysis every 10 cycles
                    if (++cycle_count >= ANALYSIS_CYCLE) {
                        std::cout << "\nPerforming comprehensive analysis..." << std::endl;
                        
                        auto grid = generateHeatmapGrid();
                        if (!grid.empty()) {
                            storeCoverageAnalysis(grid);
                            auto recommendations = recommendRouterPlacements(grid);
                            printOptimizationReport(grid, recommendations);
                        }
                        
                        performHistoricalAnalysis();
                        cycle_count = 0;
                    }
                } else {
                    std::cout << "No networks detected in this scan cycle" << std::endl;
                }

                // Clean up old data (keep only last 30 days)
                cleanupOldData();

            } catch (const std::exception& e) {
                std::cerr << "Error in monitoring cycle: " << e.what() << std::endl;
            }

            // Sleep for the specified interval
            std::this_thread::sleep_for(std::chrono::seconds(SCAN_INTERVAL));
        }
    }

private:
    void cleanupOldData() {
        std::lock_guard<std::mutex> lock(db_mutex);
        
        auto now = std::chrono::system_clock::now();
        auto cutoff = now - std::chrono::hours(24 * HISTORY_DAYS);
        auto timestamp_cutoff = std::chrono::duration_cast<std::chrono::seconds>(
            cutoff.time_since_epoch()).count();

        const char* cleanup_sql = R"(
            DELETE FROM signal_measurements WHERE timestamp < ?;
            DELETE FROM alerts WHERE timestamp < ?;
            DELETE FROM coverage_analysis WHERE timestamp < ?;
        )";

        char* err_msg = nullptr;
        sqlite3_stmt* stmt;
        
        // Cleanup signal measurements
        if (sqlite3_prepare_v2(db, "DELETE FROM signal_measurements WHERE timestamp < ?", -1, &stmt, nullptr) == SQLITE_OK) {
            sqlite3_bind_int64(stmt, 1, timestamp_cutoff);
            sqlite3_step(stmt);
            sqlite3_finalize(stmt);
        }
        
        // Cleanup alerts
        if (sqlite3_prepare_v2(db, "DELETE FROM alerts WHERE timestamp < ?", -1, &stmt, nullptr) == SQLITE_OK) {
            sqlite3_bind_int64(stmt, 1, timestamp_cutoff);
            sqlite3_step(stmt);
            sqlite3_finalize(stmt);
        }
        
        // Cleanup coverage analysis
        if (sqlite3_prepare_v2(db, "DELETE FROM coverage_analysis WHERE timestamp < ?", -1, &stmt, nullptr) == SQLITE_OK) {
            sqlite3_bind_int64(stmt, 1, timestamp_cutoff);
            sqlite3_step(stmt);
            sqlite3_finalize(stmt);
        }

        // Vacuum database to reclaim space
        sqlite3_exec(db, "VACUUM;", nullptr, nullptr, &err_msg);
        if (err_msg) {
            sqlite3_free(err_msg);
        }
    }
};

int main() {
    std::cout << "WiFi Signal Strength Mapping System v1.0" << std::endl;
    std::cout << "Corporate Network Infrastructure Optimization Tool" << std::endl;
    std::cout << "=================================================" << std::endl;

    WiFiSignalMapper mapper;
    mapper.run();

    return 0;
}
