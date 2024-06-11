/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef UTILITY_RL_PPO_HPP
#define UTILITY_RL_PPO_HPP

#include <random>
#include <torch/torch.h>

namespace utility::rl {

    struct PolicyNetwork : torch::nn::Module {
        torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};

        PolicyNetwork(int input_size, int output_size) {
            fc1 = register_module("fc1", torch::nn::Linear(input_size, 128));
            fc2 = register_module("fc2", torch::nn::Linear(128, 128));
            fc3 = register_module("fc3", torch::nn::Linear(128, output_size));
        }

        torch::Tensor forward(torch::Tensor x) {
            x = torch::relu(fc1->forward(x));
            x = torch::relu(fc2->forward(x));
            x = fc3->forward(x);
            return torch::softmax(x, -1);
        }
    };

    struct ValueNetwork : torch::nn::Module {
        torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};

        ValueNetwork(int input_size) {
            fc1 = register_module("fc1", torch::nn::Linear(input_size, 128));
            fc2 = register_module("fc2", torch::nn::Linear(128, 128));
            fc3 = register_module("fc3", torch::nn::Linear(128, 1));
        }

        torch::Tensor forward(torch::Tensor x) {
            x = torch::relu(fc1->forward(x));
            x = torch::relu(fc2->forward(x));
            x = fc3->forward(x);
            return x;
        }
    };

    class PPOAgent {
    public:
        PPOAgent(int state_dim, int action_dim)
            : policy_net(state_dim, action_dim)
            , value_net(state_dim)
            , policy_optimizer(policy_net.parameters(), torch::optim::AdamOptions(1e-3))
            , value_optimizer(value_net.parameters(), torch::optim::AdamOptions(1e-3)) {}

        void update(std::vector<torch::Tensor>& states,
                    std::vector<torch::Tensor>& actions,
                    std::vector<torch::Tensor>& rewards) {
            // TODO: Implement PPO update logic
        }

    private:
        PolicyNetwork policy_net;
        ValueNetwork value_net;
        torch::optim::Adam policy_optimizer;
        torch::optim::Adam value_optimizer;
        std::mt19937 generator{std::random_device{}()};
    };
}  // namespace utility::rl

#endif
