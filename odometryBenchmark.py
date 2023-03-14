import json

# Define a weight function that gives more weight to more recent values
def weight_function(index, n):
    return (n - index) / n  # Weight function: weights decrease linearly with index, with the most recent value having weight 1

# Load the JSON file
with open('odometryBenchmark.json', 'r') as f:
    data = json.load(f)


num_values = len(data)
rWTt_sum = 0
Rtw_sum = 0

for i, datum in enumerate(data):
    weight = (num_values - i) / num_values
    rWTt_sum += weight * ( datum["data"]["rWTt"]["x"] + datum["data"]["rWTt"]["y"] + datum["data"]["rWTt"]["z"] ) / 3
    Rtw_sum += weight * ( datum["data"]["Rtw"]["x"] + datum["data"]["Rtw"]["y"] + datum["data"]["Rtw"]["z"] ) / 3

weighted_rWTt = rWTt_sum / num_values
weighted_Rtw = Rtw_sum / num_values

print(f"Weighted rWTt: {weighted_rWTt:.6f}")
print(f"Weighted Rtw: {weighted_Rtw:.6f}")
