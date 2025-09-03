from apscheduler.schedulers.background import BackgroundScheduler
import time

def some_job():
    print("Decorated job")

scheduler = BackgroundScheduler()
job = scheduler.add_job(some_job, 'interval', seconds=1) # hours=1)
scheduler.start()

for i in range(5):
    print('yippe')
    time.sleep(1)

job.remove()
print('removed job')