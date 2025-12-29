/**
 * main.cpp
 * --------
 * Entry point for RRT* benchmark.
 *
 * Usage:
 *   ./rrt_benchmark                    # Use default map
 *   ./rrt_benchmark -m map.yaml -p 20 -i 100
 */

#include "rrt_benchmark.h"
#include <iostream>
#include <filesystem>
#include <getopt.h>

// Default map path
const std::string DEFAULT_MAP = "maps/hospital_map_cropped.yaml";

void printHelp(const char* name) {
    std::cout << "RRT* Path Planning Benchmark\n\n"
              << "Usage: " << name << " [options]\n\n"
              << "Options:\n"
              << "  -m, --map <file>       Map YAML file (default: " << DEFAULT_MAP << ")\n"
              << "  -o, --output <dir>     Output directory (default: results)\n"
              << "  -p, --pairs <n>        Number of point pairs (default: 20)\n"
              << "  -i, --iterations <n>   Iterations per pair (default: 100)\n"
              << "  -d, --distance <m>     Min distance between points (default: 5.0)\n"
              << "  -t, --timeout <s>      Planning timeout (default: 3.0)\n"
              << "  -s, --seed <n>         Random seed\n"
              << "  -q, --quiet            Less output\n"
              << "  -h, --help             Show this help\n\n"
              << "Examples:\n"
              << "  " << name << "                                    # Use default map\n"
              << "  " << name << " -m maps/my_map.yaml -p 5 -i 10\n"
              << "  " << name << " -p 20 -i 100 -t 5.0 -s 42\n";
}

int main(int argc, char** argv) {
    // Defaults
    std::string map_path = DEFAULT_MAP;
    std::string output_dir = "results";
    int pairs = 20;
    int iterations = 100;
    double min_distance = 5.0;
    double timeout = 3.0;
    int seed = -1;
    bool verbose = true;

    static struct option opts[] = {
        {"map",        required_argument, 0, 'm'},
        {"output",     required_argument, 0, 'o'},
        {"pairs",      required_argument, 0, 'p'},
        {"iterations", required_argument, 0, 'i'},
        {"distance",   required_argument, 0, 'd'},
        {"timeout",    required_argument, 0, 't'},
        {"seed",       required_argument, 0, 's'},
        {"quiet",      no_argument,       0, 'q'},
        {"help",       no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    int c;
    while ((c = getopt_long(argc, argv, "m:o:p:i:d:t:s:qh", opts, nullptr)) != -1) {
        switch (c) {
            case 'm': map_path = optarg; break;
            case 'o': output_dir = optarg; break;
            case 'p': pairs = std::atoi(optarg); break;
            case 'i': iterations = std::atoi(optarg); break;
            case 'd': min_distance = std::atof(optarg); break;
            case 't': timeout = std::atof(optarg); break;
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
        rrt_bench::Config config;
        config.planning_timeout = timeout;

        // Create runner
        rrt_bench::Runner runner(map_path, config);

        if (seed >= 0) {
            runner.setSeed(static_cast<unsigned int>(seed));
        }

        // Run benchmark
        auto session = runner.run(pairs, iterations, min_distance, verbose);
        session.map_file = map_path;

        // Save results
        std::string base = output_dir + "/benchmark_" + session.timestamp;
        runner.saveJson(session, base + ".json");
        runner.saveSummary(session, base + ".txt");

        std::cout << "\nDone! Analyze with:\n";
        std::cout << "  python3 scripts/analyze.py " << base << ".json\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}