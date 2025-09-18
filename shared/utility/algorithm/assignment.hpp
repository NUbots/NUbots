#ifndef UTILITY_ALGORITHM_ASSIGNMENT_HPP
#define UTILITY_ALGORITHM_ASSIGNMENT_HPP

#include <Eigen/Dense>
#include <map>
#include <utility>
#include <vector>

namespace utility::algorithm {

    /**
     * Solve the assignment problem using the Hungarian algorithm given a MAX_DIMS x MAX_DIMS matrix where MAX_DIMS =
     * max{W, T}. This implementation will have a time complexity of O(n^3). See
     * https://cp-algorithms.com/graph/hungarian-algorithm.html for the relevant mathematical proofs. This algorithm may
     * not agree with brute force methods if there are multiple optimal assignments, or if the pairings are imbalanced.
     *
     * @tparam cost_type         The type of the cost matrix, e.g. float, double, int
     * @tparam number_of_workers The number of workers which correspond to the number of rows in the cost matrix
     * @tparam number_of_tasks   The number of tasks which correspond to the number of columns in the cost matrix
     *
     * @param cost_matrix         The cost matrix where each element represents the cost of assigning a worker to a task
     * @return std::map<int, int> The assignment of workers to tasks
     */
    template <typename cost_type, int number_of_workers, int number_of_tasks>
    [[nodiscard("Output is only useful when it has to be used")]] inline std::map<int, int> determine_assignment(
        const Eigen::Matrix<cost_type, number_of_workers, number_of_tasks>& cost_matrix) {
        const int workers = static_cast<int>(cost_matrix.rows());
        const int tasks   = static_cast<int>(cost_matrix.cols());

        const int MAX_DIMS  = std::max(workers, tasks);
        const cost_type INF = std::numeric_limits<cost_type>::max();

        // Holds the values that have been subtracted from every element in row i (worker i mapped to these tasks)
        std::vector<cost_type> worker_potentials(MAX_DIMS + 1);
        // The value subtracted from every element in the jth column
        std::vector<cost_type> task_potentials(MAX_DIMS + 1);
        // Stores the smallest reduced cost of any path that can reach some task j from the current graph
        std::vector<cost_type> shortest_paths_to_task(MAX_DIMS + 1);
        // Storage for the current matching
        std::vector<int> task_to_worker(MAX_DIMS + 1);
        // Storage for the links when the assignment cost to worker i is improved by some task j.
        // Used for back tracking once an unmatched column is reached to augment the matching
        std::vector<int> predecessor_tasks(MAX_DIMS + 1);
        // Stores which tasks are already in the alternating tree for the current augmentation iterate.
        // Differentiates between "covered" and "uncovered" tasks (columns)
        std::vector<bool> visited_tasks(MAX_DIMS + 1);

        for (int w = 1; w <= MAX_DIMS; ++w) {
            task_to_worker[0] = w;
            std::fill(shortest_paths_to_task.begin(), shortest_paths_to_task.end(), INF);
            std::fill(visited_tasks.begin(), visited_tasks.end(), false);
            int current_task = 0;

            // Search for the smallest augmenting path
            do {
                visited_tasks[current_task] = true;
                int current_worker          = task_to_worker[current_task];
                int next_task               = 0;
                // The most recent smallest uncovered slack that will be subtracted to uncovered columns and added to
                // covered rows
                cost_type delta = INF;
                for (int t = 1; t <= MAX_DIMS; ++t) {
                    // If the task has not been visited
                    if (!visited_tasks[t]) {
                        // Look up the current slack based on the entry in the cost matrix then update it using the
                        // worker and task potentials
                        cost_type current_reduced_cost = (current_worker <= workers && t <= tasks)
                                                             ? cost_matrix(current_worker - 1, t - 1)
                                                             : cost_type(0);
                        current_reduced_cost -= worker_potentials[current_worker] + task_potentials[t];

                        // If the reduced cost for the current task is less than the shortest path to the task, update
                        // the shortest path and predecessor
                        if (current_reduced_cost < shortest_paths_to_task[t]) {
                            shortest_paths_to_task[t] = current_reduced_cost;
                            predecessor_tasks[t]      = current_task;
                        }
                        // If the shortest path to the task is smaller than the current smallest slack, update the
                        // smallest slack and next task
                        if (shortest_paths_to_task[t] < delta) {
                            delta     = shortest_paths_to_task[t];
                            next_task = t;
                        }
                    }
                }

                // Update the potentials using delta. This is the same step as adding the smallest uncovered value to
                // covered rows and subtracting it from uncovered columns. See
                // https://brilliant.org/wiki/hungarian-matching/ for the visual and matrix counterpart of this
                // algorithm
                for (int t = 0; t <= MAX_DIMS; ++t) {
                    if (visited_tasks[t]) {
                        worker_potentials[task_to_worker[t]] += delta;
                        task_potentials[t] -= delta;
                    }
                    else {
                        shortest_paths_to_task[t] -= delta;
                    }
                }

                current_task = next_task;

            } while (task_to_worker[current_task] != 0);

            // Augment along the path
            do {
                int next_task                = predecessor_tasks[current_task];
                task_to_worker[current_task] = task_to_worker[next_task];
                current_task                 = next_task;
            } while (current_task != 0);
        }

        // Build assignment map
        std::map<int, int> result;
        for (int t = 1; t <= tasks; ++t) {
            if (task_to_worker[t] != 0 && task_to_worker[t] <= workers) {
                result.emplace(task_to_worker[t] - 1, t - 1);
            }
        }

        return result;
    }

}  // namespace utility::algorithm

#endif  // UTILITY_ALGORITHM_ASSIGNMENT_HPP
