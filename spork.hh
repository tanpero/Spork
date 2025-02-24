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
#include <thread>
#include <mutex>
#include <future>
#include <cmath>
#include <format>
#include <atomic>
#include <thread>

namespace __spork
{    

    class ProgressBar {
    public:
        ProgressBar(size_t total_tasks, size_t bar_width = 50)
            : total_tasks(total_tasks), bar_width(bar_width), completed_tasks(0), running(true) {
            bar_fill = "█";
            bar_half = "▌";
            bar_fractions = {"▏", "▎", "▍", "▋", "▊", "▉"};
        }

        void start() {
            running = true;
            progress_thread = std::thread(&ProgressBar::update_progress, this);
        }
        
        void delay(int ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }

        void increment(size_t count = 1) {
            std::lock_guard<std::mutex> lock(mutex);
            completed_tasks += count;
        }

        void stop() {
            running = false;
            if (progress_thread.joinable()) {
                progress_thread.join();
            }
            std::cout << ANSI_CLEAR_LINE << "\r|" << repeat(bar_fill, bar_width) << "| 100%" << std::flush;
            std::cout << std::endl;
        }

    private:
        size_t total_tasks;

        std::atomic<size_t> completed_tasks;

        size_t bar_width;

        std::atomic<bool> running;

        std::thread progress_thread;

        std::mutex mutex;

        std::string bar_fill;
        std::string bar_half;
        std::vector<std::string> bar_fractions;

        const std::string ANSI_SAVE_CURSOR = "\033[s";
        const std::string ANSI_RESTORE_CURSOR = "\033[u";
        const std::string ANSI_CLEAR_LINE = "\033[2K";

        void update_progress() {
            while (running) {
                std::lock_guard<std::mutex> lock(mutex);
                size_t current_tasks = completed_tasks.load();

                size_t total_ticks = bar_width * 8;

                size_t completed_ticks = static_cast<size_t>((
                    static_cast<double>(current_tasks) / total_tasks) * total_ticks);

                size_t filled_length = completed_ticks / 8;
                size_t fractional_part = completed_ticks % 8;

                std::string bar = repeat(bar_fill, filled_length);

                if (fractional_part > 0) {
                    if (fractional_part < 4) {
                        bar += bar_fractions[fractional_part - 1];
                    } else if (fractional_part == 4) {
                        bar += bar_half;
                    } else {
                        bar += bar_fractions[fractional_part - 2];
                    }
                }

                size_t empty_length = bar_width - filled_length - (fractional_part > 0 ? 1 : 0);
                bar += repeat(" ", empty_length);

                double progress = static_cast<double>(current_tasks) / total_tasks;

                std::cout << ANSI_SAVE_CURSOR;

                std::cout << "\r|" << bar << "| " << std::format("{:.2f}%", progress * 100) << std::flush;

                std::cout << ANSI_RESTORE_CURSOR << std::flush;

                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
            
        std::string repeat(const std::string& str, size_t count) {
            std::string result;
            result.reserve(str.size() * count);
            for (size_t i = 0; i < count; ++i) {
                result += str;
            }
            return result;
        }

    };

}

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
        
        ProgressBar progress_bar(num_iterations, 50);
        progress_bar.start();

        for (int i = 0; i < num_iterations; ++i) {
            double runtime_ms;
            if (pre_func) {
                if constexpr (!std::is_void_v<decltype(pre_func())>) {
                    auto pre_result = pre_func();
                    auto start_time = std::chrono::high_resolution_clock::now();
                    func(pre_result);
                    auto end_time = std::chrono::high_resolution_clock::now();
                    runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                } else {
                    pre_func();
                    auto start_time = std::chrono::high_resolution_clock::now();
                    func();
                    auto end_time = std::chrono::high_resolution_clock::now();
                    runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                }
            } else {
                auto start_time = std::chrono::high_resolution_clock::now();
                func();                
                auto end_time = std::chrono::high_resolution_clock::now();
                runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
            }
            runtimes.push_back(runtime_ms);
            progress_bar.increment();
        }
        progress_bar.stop();
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


    template <typename Func, typename PreFunc = std::function<void()>>
    double RunBenchmarkWithMultithread(const Func& func, int num_iterations, int num_threads,
                                    double& min_time, double& max_time,
                                    double& std_dev, size_t& memory_usage,
                                    const PreFunc& pre_func = nullptr) {
        std::vector<double> runtimes;
        std::mutex mutex;

        auto thread_func = [&](int start, int end) {

            std::vector<double> local_runtimes;
            for (int i = start; i < end; ++i) {
                double runtime_ms;
                if (pre_func) {
                    if constexpr (!std::is_void_v<decltype(pre_func())>) {
                        auto pre_result = pre_func();
                        auto start_time = std::chrono::high_resolution_clock::now();
                        func(pre_result);
                        auto end_time = std::chrono::high_resolution_clock::now();
                        runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                    } else {
                        pre_func();
                        auto start_time = std::chrono::high_resolution_clock::now();
                        func();
                        auto end_time = std::chrono::high_resolution_clock::now();
                        runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                    }
                } else {
                    auto start_time = std::chrono::high_resolution_clock::now();
                    func();
                    auto end_time = std::chrono::high_resolution_clock::now();
                    runtime_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
                }

                std::lock_guard<std::mutex> lock(mutex);
                local_runtimes.push_back(runtime_ms);
            }

            std::lock_guard<std::mutex> lock(mutex);
            runtimes.insert(runtimes.end(), local_runtimes.begin(), local_runtimes.end());
        };

        std::vector<std::thread> threads;
        int iterations_per_thread = num_iterations / num_threads;
        for (int i = 0; i < num_threads; ++i) {
            int start = i * iterations_per_thread;
            int end = (i == num_threads - 1) ? num_iterations : start + iterations_per_thread;
            threads.emplace_back(thread_func, start, end);
        }

        for (auto& thread : threads) {
            thread.join();
        }

        min_time = *std::min_element(runtimes.begin(), runtimes.end());
        max_time = *std::max_element(runtimes.begin(), runtimes.end());
        double average_time = std::accumulate(runtimes.begin(), runtimes.end(), 0.0) / static_cast<double>(num_iterations);

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
        runtime_ns = std::floor(runtime_ns);
        result << runtime_ns << "ns";

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

    template <typename Func, typename PreFunc = std::function<void()>>
    void Benchmark(Func&& func, int count, int num_threads, PreFunc pre_func = nullptr) {
        double min_runtime, max_runtime, std_dev;
        size_t memory_usage;
        double average_runtime = RunBenchmarkWithMultithread(func, count, num_threads, std::ref(min_runtime),
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

template <typename Func, typename PreFunc = std::function<void()>>
void benchmark(Func&& func, int count, int num_threads, PreFunc pre_func = nullptr) {
    __spork::Benchmark(std::forward<Func>(func), count, num_threads, std::forward<PreFunc>(pre_func));
}

#endif // !_BENCHMARK_HPP_
