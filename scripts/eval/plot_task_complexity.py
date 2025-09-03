import argparse

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

sns.set_theme()


# Plotting function
def plot_data(data, title):
    plt.figure(figsize=(8, 5))

    for method, results in data.items():
        steps = sorted(results.keys())
        means = [np.mean(results[step]) * 100 for step in steps]
        stds = [np.std(results[step]) * 100 for step in steps]
        n = len(next(iter(results.values())))  # Number of samples
        conf_intervals = [1.96 * (std / np.sqrt(n)) for std in stds]

        plt.plot(steps, means, label=method)
        plt.fill_between(
            steps,
            np.array(means) - np.array(conf_intervals),
            np.array(means) + np.array(conf_intervals),
            alpha=0.2,
        )

    plt.title(f"Task {title} Success Rate vs. Task Complexity")
    plt.xlabel("Steps in Optimal Path")
    plt.ylabel("Average % Success Rate")
    plt.ylim(bottom=-5.0)
    plt.legend()
    plt.grid(True)
    plt.show()


def main(args):
    if args.mode == "planning":
        assert (
            args.experiment == "block_stacking"
        ), "Only have planning success data for blocks"
        planning = {
            "Ours": {
                1: [1.0, 1.0, 1.0, 1.0, 1.0],
                2: [1.0, 1.0, 1.0, 1.0, 1.0],
                3: [8 / 9, 8 / 9, 8 / 9, 7 / 9, 9 / 9],
                4: [0.5, 0.5, 0.75, 0.75, 0.5],
            },
            "No Object Suggester": {
                1: [1.0, 1.0, 1.0, 1.0, 1.0],
                2: [1.0, 1.0, 1.0, 1.0, 1.0],
                3: [6 / 9, 6 / 9, 9 / 9, 9 / 9, 8 / 9],
                4: [0.5, 0.5, 0.75, 0.25, 0.75],
            },
            "Greedy": {
                1: [1.0, 1.0, 1.0, 1.0, 1.0],
                2: [3 / 8, 6 / 8, 6 / 8, 5 / 8, 6 / 8],
                3: [1 / 9, 1 / 9, 0 / 9, 2 / 9, 2 / 9],
                4: [0.0, 0.0, 0.0, 0.0, 0.0],
            },
            "Random Rollouts": {
                1: [1.0, 1.0, 1.0, 1.0, 1.0],
                2: [3 / 8, 4 / 8, 4 / 8, 2 / 8, 3 / 8],
                3: [0.0, 0.0, 0.0, 0.0, 3 / 9],
                4: [0.0, 0.0, 0.0, 0.0, 1 / 4],
            },
        }
        plot_data(planning, "Planning")

    # Execution data
    else:
        if args.experiment == "block_stacking":
            execution = {
                "Ours": {
                    1: [1.0, 1.0, 1.0, 1.0, 1.0],
                    2: [7 / 8, 6 / 8, 6 / 8, 5 / 8, 7 / 8],
                    3: [4 / 9, 6 / 9, 5 / 9, 6 / 9, 5 / 9],
                    4: [0.0, 0.5, 0.5, 0.25, 0.25],
                },
                "No Object Suggester": {
                    1: [1.0, 1.0, 1.0, 1.0, 1.0],
                    2: [7 / 8, 6 / 8, 6 / 8, 8 / 8, 7 / 8],
                    3: [3 / 9, 2 / 9, 3 / 9, 5 / 9, 5 / 9],
                    4: [0.5, 0.25, 0.5, 0.0, 0.5],
                },
                "Greedy": {
                    1: [1.0, 1.0, 1.0, 1.0, 1.0],
                    2: [2 / 8, 3 / 8, 5 / 8, 4 / 8, 3 / 8],
                    3: [0 / 9, 1 / 9, 0 / 9, 1 / 9, 1 / 9],
                    4: [0.0, 0.0, 0.0, 0.0, 0.0],
                },
                "Random Rollouts": {
                    1: [1.0, 1.0, 1.0, 1.0, 1.0],
                    2: [2 / 8, 3 / 8, 3 / 8, 2 / 8, 2 / 8],
                    3: [0.0, 0.0, 0.0, 0.0, 2 / 9],
                    4: [0.0, 0.0, 0.0, 0.0, 0.0],
                },
            }
        elif args.experiment == "table_bussing":
            execution = {
                "Ours": {
                    1: [1.0],
                    2: [1.0],
                    3: [1.0],
                    4: [0.75],
                    5: [0.67],
                },
                "No Object Suggester": {
                    1: [1.0],
                    2: [0.0],
                    3: [0.8],
                    4: [0.75],
                    5: [0.33],
                },
                "Greedy": {
                    1: [1.0],
                    2: [0.0],
                    3: [0.2],
                    4: [0.0],
                    5: [0.0],
                },
                "Random Rollouts": {
                    1: [0.0],
                    2: [1.0],
                    3: [0.0],
                    4: [0.25],
                    5: [0.0],
                },
            }

        plot_data(execution, "Execution")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot task planning or execution success vs. task complexity"
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["planning", "execution"],
        default="execution",
        help="Plot planning or execution success",
    )
    parser.add_argument(
        "--experiment",
        type=str,
        choices=["block_stacking", "table_bussing"],
        default="block_stacking",
        help="Environment",
    )
    args = parser.parse_args()

    main(args)
