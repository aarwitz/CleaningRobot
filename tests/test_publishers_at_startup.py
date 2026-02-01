import subprocess
import time

def get_publishing_rate(topic, duration=3):
    """
    Uses ros2 topic hz to get the publishing rate of a topic inside the docker container.
    """
    cmd = f'docker exec docker-vision-1 bash -c "source /opt/vision_ws/install/setup.bash && timeout {duration} ros2 topic hz {topic}"'
    try:
        output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, text=True)
        return output
    except subprocess.CalledProcessError as e:
        return e.output

def get_active_topics():
    """
    Uses ros2 topic list to get all active topics inside the docker container.
    """
    cmd = 'docker exec docker-vision-1 bash -c "source /opt/vision_ws/install/setup.bash && ros2 topic list"'
    try:
        output = subprocess.check_output(cmd, shell=True, stderr=subprocess.STDOUT, text=True)
        topics = output.strip().split('\n')
        return topics
    except subprocess.CalledProcessError as e:
        return []

def main():
    print("Getting active topics...")
    topics = get_active_topics()
    if not topics:
        print("No topics found or failed to get topics.")
        return
    print(f"Found {len(topics)} topics. Checking publishing rates...")
    for topic in topics:
        print(f"\nTopic: {topic}")
        rate_info = get_publishing_rate(topic)
        print(rate_info)
        time.sleep(1)  # avoid spamming the container

if __name__ == "__main__":
    main()
