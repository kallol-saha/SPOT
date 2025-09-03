import argparse
import wandb

def combine_runs(wandb_entity: str, wandb_project: str, new_run_name: str, max_samples: int, run_ids: list):
    """Combine runs from wandb into a new run.
    
    Args:
        wandb_entity (str): The entity name of the project.
        wandb_project (str): The project name.
        new_run_name (str): The name of the new run.
        max_samples (int): The maximum number of samples to log.
        run_ids (list): The ids of the runs to combine.
    """
    wandb.init(project=wandb_project, entity=wandb_entity, name=new_run_name)
    
    last_step = 0
    last_global_step = 0
    for run_id in run_ids:
        api = wandb.Api()
        run = api.run(f"{wandb_entity}/{wandb_project}/{run_id}")
        
        history = run.history(samples=max_samples, pandas=False)
        
        # Adjust step values and log to the new run
        max_step = 0
        max_global_step = 0
        for record in history:
            record['_step'] += last_step
            record['trainer/global_step'] += last_global_step
            max_step = max(max_step, record['_step'])
            max_global_step = max(max_global_step, record['trainer/global_step'])
            wandb.log(record, step=int(record['_step']))
        
        # Update the last step for the next iteration
        last_step = max_step
        last_global_step = max_global_step

    # Finish the new run
    wandb.finish()

if __name__ == '__main__':
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--wandb_entity', type=str, required=True)
    argparser.add_argument('--wandb_project', type=str, required=True)
    argparser.add_argument('--new_run_name', type=str, required=True)
    argparser.add_argument('--max_samples', type=int, default=1000000)
    argparser.add_argument('--run_ids', nargs='+', type=str, required=True)
    
    args = argparser.parse_args()
    
    combine_runs(args.wandb_entity, args.wandb_project, args.new_run_name, args.max_samples, args.run_ids)