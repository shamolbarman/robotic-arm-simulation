import google.generativeai as genai
import json

# 1. Configure your Gemini API Key here (From Google AI Studio)
API_KEY = "YOUR_GEMINI_API_KEY_HERE"
genai.configure(api_key=API_KEY)

# 2. Setup Gemini Model Configuration
model = genai.GenerativeModel('gemini-1.5-flash')

def generate_robot_plan(user_command):
    """
    Transforms human natural language commands into structured robotic JSON tasks.
    """
    prompt = f"""
    You are an AI Robotics Orchestrator. 
    Convert the following human command into a structured JSON format for a robot.
    
    Command: "{user_command}"
    
    Expected JSON Structure:
    {{
      "task_name": "Name of the task",
      "actions": ["list", "of", "steps"],
      "priority": "high/medium/low"
    }}
    """
    
    try:
        # Generate response from Gemini API
        response = model.generate_content(prompt)
        return response.text
    except Exception as e:
        return f"Error encountered: {str(e)}"

# 3. Main execution and testing
if __name__ == "__main__":
    print("--- Robo-Ops AI Orchestrator ---")
    
    # Test command for the robotic arm
    command = "Pick up the blue box from Zone A and move it to Zone C"
    
    print(f"User Input: {command}")
    print("Generating Robotic Plan via Gemini...")
    
    result = generate_robot_plan(command)
    print("\nGenerated JSON Plan:")
    print(result)
