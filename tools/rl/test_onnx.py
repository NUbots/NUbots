import onnxruntime
import numpy as np
import time
import argparse
import json
from pathlib import Path
from utility.dockerise import run_on_docker

class OnnxInfer:
    def __init__(self, onnx_model_path, input_name="obs", awd=False):
        self.onnx_model_path = onnx_model_path
        self.ort_session = onnxruntime.InferenceSession(
            self.onnx_model_path, providers=["CPUExecutionProvider"]
        )
        self.input_name = input_name
        self.awd = awd

    def infer(self, inputs):
        if self.awd:
            outputs = self.ort_session.run(None, {self.input_name: [inputs]})
            return outputs[0][0]
        else:
            outputs = self.ort_session.run(
                None, {self.input_name: inputs.astype("float32")}
            )
            return outputs[0]

@run_on_docker
def register(command):
    command.description = "Test an ONNX model with dummy values"
    command.add_argument("--onnx_model_path", type=str, required=True, help="Path to the ONNX model file")
    command.add_argument("--num_tests", type=int, default=1000, help="Number of test iterations")
    command.add_argument("--obs_size", type=int, default=49, help="Size of observation vector")
    command.add_argument("--verify_data", action="store_true", help="Verify against example data")

@run_on_docker
def run(**kwargs):
    """
    Tests an ONNX model with random inputs and measures inference time.
    """
    onnx_model_path = kwargs.get("onnx_model_path")
    num_tests = kwargs.get("num_tests", 1000)
    obs_size = kwargs.get("obs_size", 49)
    verify_data = kwargs.get("verify_data", False)

    # Verify model exists
    model_path = Path(onnx_model_path)
    if not model_path.exists():
        print(f"Error: ONNX model not found at {onnx_model_path}")
        return

    print(f"Testing ONNX model: {onnx_model_path}")
    print(f"Observation size: {obs_size}")
    print(f"Number of test iterations: {num_tests}")

    # Initialize ONNX inference
    oi = OnnxInfer(onnx_model_path, awd=True)

    # Verify against example data if requested
    if verify_data:
        try:
            with open("recordings/example_data.json", "r") as f:
                example_data = json.load(f)

            print("\nVerifying against example data:")
            print("Loading observation:", example_data["observation"])
            print("Expected action:", example_data["action"])

            # Convert to numpy array and run inference
            inputs = np.array(example_data["observation"], dtype=np.float32)
            output = oi.infer(inputs)

            print("Actual action:", output)

            # Calculate error
            error = np.abs(output - np.array(example_data["action"]))
            print("Maximum absolute error:", np.max(error))
            print("Mean absolute error:", np.mean(error))

            if np.max(error) < 1e-5:
                print("Verification successful! Outputs match within tolerance.")
            else:
                print("Warning: Outputs differ from expected values.")

        except FileNotFoundError:
            print("Error: example_data.json not found. Run RLWalk first to generate example data.")
            return
        except json.JSONDecodeError:
            print("Error: Invalid JSON in example_data.json")
            return

    # Run tests with random inputs
    times = []
    for i in range(num_tests):
        # Generate random input
        inputs = np.random.uniform(size=obs_size).astype(np.float32)

        # Time inference
        start = time.time()
        output = oi.infer(inputs)
        inference_time = time.time() - start
        times.append(inference_time)

        # Print first few results
        if i < 3:
            print(f"\nTest {i + 1}:")
            print(f"Input shape: {inputs.shape}")
            print(f"Output shape: {output.shape}")
            print(f"Output sample: {output[:5]}")  # Print first 5 values
            print(f"Inference time: {inference_time*1000:.2f}ms")

    # Calculate and print statistics
    avg_time = sum(times) / len(times)
    print("\nPerformance Statistics:")
    print(f"Average inference time: {avg_time*1000:.2f}ms")
    print(f"Average FPS: {1/avg_time:.2f}")
    print(f"Min time: {min(times)*1000:.2f}ms")
    print(f"Max time: {max(times)*1000:.2f}ms")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, required=True)
    parser.add_argument("--num_tests", type=int, default=1000)
    parser.add_argument("--obs_size", type=int, default=46)
    parser.add_argument("--verify_data", action="store_true")
    args = parser.parse_args()

    run(onnx_model_path=args.onnx_model_path,
        num_tests=args.num_tests,
        obs_size=args.obs_size,
        verify_data=args.verify_data)
