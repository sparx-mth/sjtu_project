/**
 * rrt_first_solution_main.cpp
 * ---------------------------
 * Entry point for RRT first-solution benchmark.
 *
 * Usage:
 *   ./rrt_first_benchmark                          # Use defaults
 *   ./rrt_first_benchmark -m map.yaml -k 20 -i 100 # 20 pairs, 100 iterations
 *   ./rrt_first_benchmark -k 10 -i 50 -d 30 -t 4   # Custom settings
 */

#include "rrt_first_solution.h"
#include <iostream>
#include <filesystem>
#include <getopt.h>
#include <ompl/util/Console.h>

// Default map path
const std::string DEFAULT_MAP = "maps/hospital_map_cropped.yaml";

void printHelp(const char* name) {
    std::cout << "RRT First-Solution Path Planning Benchmark\n\n"
              << "Usage: " << name << " [options]\n\n"
              << "This benchmark uses regular RRT (not RRT*) to measure:\n"
              << "  - Time to find the FIRST valid path (no optimization)\n"
              << "  - Path characteristics after smoothing and interpolation\n\n"
              << "Options:\n"
              << "  -m, --map <file>       Map YAML file (default: " << DEFAULT_MAP << ")\n"
              << "  -o, --output <dir>     Output directory (default: results)\n"
              << "  -k, --pairs <n>        Number of point pairs K (default: 20)\n"
              << "  -i, --iterations <n>   Iterations per pair I (default: 100)\n"
              << "  -d, --distance <m>     Min air distance M between points (default: 30.0)\n"
              << "  -t, --timeout <s>      Planning timeout (default: 4.0, skip if no solution)\n"
              << "  -g, --goal-bias <f>    RRT goal bias (default: 0.05)\n"
              << "  -r, --range <f>        RRT step range (default: auto)\n"
              << "  -s, --seed <n>         Random seed for reproducibility\n"
              << "  -q, --quiet            Less output\n"
              << "  -h, --help             Show this help\n\n"
              << "Output:\n"
              << "  - JSON file with complete data (all routes, all iterations)\n"
              << "  - TXT summary with statistics\n"
              << "  - CSV file for easy analysis\n\n"
              << "Examples:\n"
              << "  " << name << "                                    # Use defaults\n"
              << "  " << name << " -m maps/my_map.yaml -k 5 -i 10\n"
              << "  " << name << " -k 20 -i 100 -d 30 -t 4 -s 42       # Reproducible run\n"
              << "  " << name << " -k 50 -i 200 -g 0.1                 # More aggressive goal bias\n";
}

int main(int argc, char** argv) {
    // Suppress OMPL debug output
    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);

    // Defaults
    std::string map_path = DEFAULT_MAP;
    std::string output_dir = "results";
    int pairs = 20;           // K point pairs
    int iterations = 100;     // I iterations per pair
    double min_distance = 30.0;  // M meters minimum air distance
    double timeout = 4.0;     // Skip if no solution found
    double goal_bias = 0.05;
    double rrt_range = 0.0;   // Auto
    int seed = -1;
    bool verbose = true;

    static struct option opts[] = {
        {"map",        required_argument, 0, 'm'},
        {"output",     required_argument, 0, 'o'},
        {"pairs",      required_argument, 0, 'k'},
        {"iterations", required_argument, 0, 'i'},
        {"distance",   required_argument, 0, 'd'},
        {"timeout",    required_argument, 0, 't'},
        {"goal-bias",  required_argument, 0, 'g'},
        {"range",      required_argument, 0, 'r'},
        {"seed",       required_argument, 0, 's'},
        {"quiet",      no_argument,       0, 'q'},
        {"help",       no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "m:o:k:i:d:t:g:r:s:qh", opts, nullptr)) != -1) {
        switch (c) {
            case 'm': map_path = optarg; break;
            case 'o': output_dir = optarg; break;
            case 'k': pairs = std::atoi(optarg); break;
            case 'i': iterations = std::atoi(optarg); break;
            case 'd': min_distance = std::atof(optarg); break;
            case 't': timeout = std::atof(optarg); break;
            case 'g': goal_bias = std::atof(optarg); break;
            case 'r': rrt_range = std::atof(optarg); break;
            case 's': seed = std::atoi(optarg); break;
            case 'q': verbose = false; break;
            case 'h': printHelp(argv[0]); return 0;
            default:  printHelp(argv[0]); return 1;
        }
    }

    try {
        // Create output directory
        std::filesystem::create_directories(output_dir);

        // Setup config
        rrt_first::Config config;
        config.planning_timeout = timeout;
        config.rrt_goal_bias = goal_bias;
        config.rrt_range = rrt_range;

        if (verbose) {
            std::cout << "========================================\n";
            std::cout << "RRT First-Solution Benchmark\n";
            std::cout << "========================================\n";
            std::cout << "Map: " << map_path << "\n";
            std::cout << "Point pairs (K): " << pairs << "\n";
            std::cout << "Iterations (I): " << iterations << "\n";
            std::cout << "Min distance (M): " << min_distance << "m\n";
            std::cout << "Timeout: " << timeout << "s\n";
            std::cout << "Goal bias: " << goal_bias << "\n";
            if (seed >= 0) std::cout << "Seed: " << seed << "\n";
            std::cout << "========================================\n\n";
        }

        // Create runner
        rrt_first::Runner runner(map_path, config);

        if (seed >= 0) {
            runner.setSeed(static_cast<unsigned int>(seed));
        }

        // Run benchmark
        auto session = runner.run(pairs, iterations, min_distance, verbose);
        session.map_file = map_path;

        // Save results
        std::string base = output_dir + "/rrt_first_" + session.timestamp;
        runner.saveJson(session, base + ".json");
        runner.saveSummary(session, base + ".txt");
        runner.saveCsv(session, base + ".csv");

        std::cout << "\nResults saved to:\n";
        std::cout << "  " << base << ".json (complete data)\n";
        std::cout << "  " << base << ".txt  (summary)\n";
        std::cout << "  " << base << ".csv  (for analysis)\n";
        std::cout << "\nVisualize routes with:\n";
        std::cout << "  python3 scripts/visualize_first_routes.py " << base << ".json\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}