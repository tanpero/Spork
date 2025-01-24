#ifndef _BENCHMARK_HPP_
#define _BENCHMARK_HPP_

/*
 * An extended benchmark framework with optional pre-function support.
 *
 * Usage:
 *   Benchmark([]() { std::sin(1) * std::cos(1); }, 100);  // Normal benchmark
 *   Benchmark([](int x) { std::sin(x) * std::cos(x); }, 100, []() { return 1; });  // With pre_func
 *   Benchmark([]() { std::sin(1) * std::cos(1); }, 100, []() { std::cout << "Pre-function executed.\n"; });  // pre_func with void return
 *
 * Written by Camille.
 *   2023-9-14
 */

#include <iostream>
#include <sstream>
#include <cmath>
#include <chrono>
#include <vector>
#include <functional>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include <type_traits>
#include <optional>

namespace __spork
{
    using BenchmarkFunction = std::function<void()>;

#ifdef _WIN32
    #include <windows.h>
    #include <psapi.h>

    size_t GetMemoryUsage() {
        PROCESS_MEMORY_COUNTERS_EX pmc;
        GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc));
        return pmc.PrivateUsage;
    }
#else
    #include <sys/resource.h>

    size_t GetMemoryUsage() {
        struct rusage rusage;
        getrusage(RUSAGE_SELF, &rusage);
        return static_cast<size_t>(rusage.ru_maxrss) * 1024;
    }
#endif

    template <typename Func, typename PreFunc = std::function<void()>>
    double RunBenchmark(const Func& func, int num_iterations,
                        double& min_time, double& max_time,
                        double& std_dev, size_t& memory_usage,
                        const PreFunc& pre_func = nullptr) {
        std::vector<double> runtimes;

        for (int i = 0; i < num_iterations; ++i) {
            if (pre_func) {
                if constexpr (!std::is_void_v<decltype(pre_func())>) {
                    auto pre_result = pre_func();
                    func(pre_result);
                } else {
                    pre_func();
                    func();
                }
            } else {
                func();
            }

            auto start_time = std::chrono::high_resolution_clock::now();
            if (pre_func) {
                if constexpr (!std::is_void_v<decltype(pre_func())>) {
                    auto pre_result = pre_func();
                    func(pre_result);
                } else {
                    pre_func();
                    func();
                }
            } else {
                func();
            }
            auto end_time = std::chrono::high_resolution_clock::now();

            double runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            runtimes.push_back(runtime_ms);
        }

        min_time = *std::min_element(runtimes.begin(), runtimes.end());
        max_time = *std::max_element(runtimes.begin(), runtimes.end());
        double average_time = std::accumulate(
                    runtimes.begin(), runtimes.end(), 0.0) / static_cast<double>(num_iterations);

        double sum_squared_diff = 0.0;
        for (double runtime : runtimes) {
            sum_squared_diff += (runtime - average_time) * (runtime - average_time);
        }
        std_dev = std::sqrt(sum_squared_diff / static_cast<double>(num_iterations));

        memory_usage = GetMemoryUsage();

        return average_time;
    }

    std::string formatDuration(double milliseconds) {
        double runtime_ms = std::floor(milliseconds);
        double runtime_us = (milliseconds - runtime_ms) * 1000;
        double runtime_ns = (runtime_us - std::floor(runtime_us)) * 1000;

        std::ostringstream result;
        if (runtime_ms > 0) {
            result << runtime_ms << "ms ";
        }
        if (runtime_us > 0) {
            runtime_us = std::floor(runtime_us);
            result << runtime_us << "μs ";
        }
        if (runtime_ns > 0) {
            runtime_ns = std::floor(runtime_ns);
            result << runtime_ns << "ns";
        }

        std::string output = result.str();
        if (!output.empty() && output.back() == ' ') {
            output.pop_back();
        }

        return output;
    }

    template <typename Func, typename PreFunc = std::function<void()>>
    void Benchmark(Func&& func, int count, PreFunc pre_func = nullptr) {
        double min_runtime, max_runtime, std_dev;
        size_t memory_usage;
        double average_runtime = RunBenchmark(func, count, std::ref(min_runtime),
                                                std::ref(max_runtime), std::ref(std_dev),
                                                std::ref(memory_usage), pre_func);

        std::cout << "\n\n-------------------------------\n";
        std::cout << "Average Runtime: " << formatDuration(average_runtime) << "\n";
        std::cout << "Minimum Runtime: " << formatDuration(min_runtime) << "\n";
        std::cout << "Maximum Runtime: " << formatDuration(max_runtime) << "\n";
    }
}

template <typename Func, typename PreFunc = std::function<void()>>
void benchmark(Func&& func, int count, PreFunc pre_func = nullptr) {
    __spork::Benchmark(std::forward<Func>(func), count, std::forward<PreFunc>(pre_func));
}

#endif // !_BENCHMARK_HPP_
