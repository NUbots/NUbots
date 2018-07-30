#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

inline std::vector<std::string>& split(const std::string& str, char delimeter, std::vector<std::string>& elements) {
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delimeter)) {
        elements.push_back(item);
    }
    return elements;
}

inline std::vector<std::string> split(const std::string& string, char delimeter) {
    std::vector<std::string> elements;
    split(string, delimeter, elements);
    return elements;
}

inline void trimLeft(std::string& str, const std::string& tokens) {
    str.erase(0, str.find_first_not_of(tokens));  // remove tokens from the beginning of the string.
}

inline void trimRight(std::string& str, const std::string& tokens) {
    str.erase(str.find_last_not_of(tokens), str.length() - 1);  // remove tokens from the beginning of the string.
}

inline std::string trim(const std::string& str, const std::string& tokens = " ") {
    std::string temp(str);
    trimLeft(temp, tokens);
    trimRight(temp, tokens);
    return temp;
}

Eigen::Matrix<float, 1, 4> softmax(const Eigen::Matrix<float, 1, 4>& x) {
    auto left_expx  = x.leftCols<2>().array().exp().matrix();
    auto right_expx = x.rightCols<2>().array().exp().matrix();
    Eigen::Matrix<float, 1, 4> ret;
    ret << left_expx / left_expx.sum(), right_expx / right_expx.sum();
    return ret;
}

Eigen::Vector4i threshold(const Eigen::Matrix<float, 1, 4>& x) {
    int left_down  = x(0) > x(1);
    int right_down = x(2) > x(3);
    Eigen::Vector4i t;
    t << left_down, 1 - left_down, right_down, 1 - right_down;
    return t;
}

int main(void) {
    YAML::Node network = YAML::LoadFile("best_vloss.yaml");

    Eigen::Matrix<float, 1, 12> input;

    Eigen::Matrix<float, 12, 8> W1;
    Eigen::Matrix<float, 1, 8> b1;
    Eigen::Matrix<float, 8, 8> W2;
    Eigen::Matrix<float, 1, 8> b2;
    Eigen::Matrix<float, 8, 4> W3;
    Eigen::Matrix<float, 1, 4> b3;

    // Initialize network
    for (size_t row = 0; row < network[0]["weights"].size(); row++) {
        for (size_t col = 0; col < network[0]["weights"][row].size(); col++) {
            W1(row, col) = network[0]["weights"][row][col].as<float>();
        }
    }
    for (size_t row = 0; row < network[0]["biases"].size(); row++) {
        b1(row) = network[0]["biases"][row].as<float>();
    }

    for (size_t row = 0; row < network[1]["weights"].size(); row++) {
        for (size_t col = 0; col < network[1]["weights"][row].size(); col++) {
            W2(row, col) = network[1]["weights"][row][col].as<float>();
        }
    }
    for (size_t row = 0; row < network[1]["biases"].size(); row++) {
        b2(row) = network[1]["biases"][row].as<float>();
    }

    for (size_t row = 0; row < network[2]["weights"].size(); row++) {
        for (size_t col = 0; col < network[2]["weights"][row].size(); col++) {
            W3(row, col) = network[2]["weights"][row][col].as<float>();
        }
    }
    for (size_t row = 0; row < network[2]["biases"].size(); row++) {
        b3(row) = network[2]["biases"][row].as<float>();
    }

    auto SELU = [](float x) -> float {
        static constexpr float alpha  = 1.6732632423543772848170429916717;
        static constexpr float lambda = 1.0507009873554804934193349852946;
        static constexpr float la     = lambda * alpha;

        if (x < 0) {
            return la * std::exp(x) - la;
        }
        else {
            return lambda * x;
        }
    };

    // Load input
    std::ifstream input_data("walk_load_datasets/long_walk.csv");
    std::string line;
    std::getline(input_data, line);
    Eigen::Matrix2i confusion = Eigen::Matrix2i::Zero();
    size_t count              = 0;
    while (std::getline(input_data, line)) {
        auto data = split(line, ' ');
        for (size_t row = 2; row < data.size(); row++) {
            input(row - 2) = std::stof(trim(data[row]));
        }

        // Perform multiplications
        auto out         = softmax(((input * W1 + b1).unaryExpr(SELU) * W2 + b2).unaryExpr(SELU) * W3 + b3);
        auto thresholded = threshold(out);

        int left_down  = std::stof(trim(data[0], ",")) < std::stof(trim(data[1])) + 0.0065f;
        int right_down = std::stof(trim(data[1], ",")) < std::stof(trim(data[0])) + 0.0065f;
        std::cout << "Batch: " << count++ << " -> "
                  << "Truth: [" << left_down << ", " << 1 - left_down << ", " << right_down << ", " << 1 - right_down
                  << "] Prediction: " << out << " Thresholded: " << thresholded.transpose() << std::endl;

        // Left
        if ((left_down == 1) && (thresholded(0) == 1) && (thresholded(1) == 0)) {
            // TP
            confusion(0, 0) += 1;
        }
        else if ((left_down == 0) && (thresholded(0) == 0) && (thresholded(1) == 1)) {
            // TN
            confusion(1, 1) += 1;
        }
        else if ((left_down == 1) && (thresholded(0) == 0) && (thresholded(1) == 1)) {
            // FN
            confusion(1, 0) += 1;
        }
        else if ((left_down == 0) && (thresholded(0) == 1) && (thresholded(1) == 0)) {
            // FP
            confusion(0, 1) += 1;
        }

        // Right
        if ((right_down == 1) && (thresholded(2) == 1) && (thresholded(3) == 0)) {
            // TP
            confusion(0, 0) += 1;
        }
        else if ((right_down == 0) && (thresholded(2) == 0) && (thresholded(3) == 1)) {
            // TN
            confusion(1, 1) += 1;
        }
        else if ((right_down == 1) && (thresholded(2) == 0) && (thresholded(3) == 1)) {
            // FN
            confusion(1, 0) += 1;
        }
        else if ((right_down == 0) && (thresholded(2) == 1) && (thresholded(3) == 0)) {
            // FP
            confusion(0, 1) += 1;
        }
    }

    std::cout << confusion << std::endl;
    std::cout << "TPR: " << confusion(0, 0) / float(confusion.leftCols<1>().sum()) << std::endl;
    std::cout << "TNR: " << confusion(1, 1) / float(confusion.rightCols<1>().sum()) << std::endl;
    std::cout << "PPV: " << confusion(0, 0) / float(confusion.topRows<1>().sum()) << std::endl;
    std::cout << "NPV: " << confusion(1, 1) / float(confusion.bottomRows<1>().sum()) << std::endl;
    std::cout << "ACC: " << (confusion(0, 0) + confusion(1, 1)) / float(confusion.sum()) << std::endl;
    std::cout << "F1.: " << (2 * confusion(0, 0)) / float(2 * confusion(0, 0) + confusion(0, 1) + confusion(1, 0))
              << std::endl;
    std::cout << "MCC: "
              << (confusion(0, 0) * confusion(1, 1) - confusion(0, 1) * confusion(1, 0))
                     / std::sqrt(float(confusion.topRows<1>().sum()) * float(confusion.bottomRows<1>().sum())
                                 * float(confusion.leftCols<1>().sum()) * float(confusion.rightCols<1>().sum()))
              << std::endl;

    return 0;
}
