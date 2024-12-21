import pandas as pd
import symfit

def fourier_series(x, f, n=0):
    """
    Returns a symbolic fourier series of order `n`.

    :param n: Order of the fourier series.
    :param x: Independent variable
    :param f: Frequency of the fourier series
    """
    # Make the parameter objects for all the terms
    a0, *cos_a = symfit.parameters(','.join(['a{}'.format(i) for i in range(0, n + 1)]))
    sin_b = symfit.parameters(','.join(['b{}'.format(i) for i in range(1, n + 1)]))
    # Construct the series
    series = a0 + sum(ai * symfit.cos(i * f * x) + bi * symfit.sin(i * f * x)
                     for i, (ai, bi) in enumerate(zip(cos_a, sin_b), start=1))
    return series

def register(command):
    command.description = "Decode an nbs file and extract any compressed jpeg files into jpeg files"

    # Command arguments
    command.add_argument("file", help="The csv file with the position data")
    command.add_argument("--output", "-o", default="recordings", help="The folder to create the videos in")

def run(file, output, **kwargs):
    # Open the file
    df = pd.read_csv(file)

    # Sort the data frame by timestamp
    df = df.sort_values(by="timestamp")

    ts_data = (df["timestamp"] - df["timestamp"].iloc[0]) / 1e9

    print(ts_data)

    with open("position_models.yaml", "w") as f:
        f.write("# Fourier series models for the position data - theta = a0 + sum(ai * sin(i*w*t) + bi * cos(i*w*t))\n")
        for col in df.columns:
            if "head" not in col and col != "timestamp":
                position_data = df[col].to_numpy()

                # Fit a Fourier series to our data
                t, theta = symfit.variables('x, y')
                w, = symfit.parameters('w')
                model_dict = {theta: fourier_series(t, f=w, n=3)}

                print(model_dict)

                fit = symfit.Fit(model_dict, x=ts_data, y=position_data)
                fit_result = fit.execute()

                print(f"Column: {col}")
                print(fit_result)

                # Write the model to the yaml file
                f.write(f"{col}:\n")
                for param, value in fit_result.params.items():
                    print(f"{param}: {value}")
                    f.write(f"  {param}: {value}\n")
                f.write("\n")
