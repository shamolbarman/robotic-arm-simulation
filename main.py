import google.generativeai as genai
import os

def robo_ops_orchestrator():
    # 1. API Configuration
    # Replace the string below with your actual Gemini API Key
    api_key = "YOUR_GEMINI_API_KEY_HERE" 
    genai.configure(api_key=api_key)

    # 2. Model Initialization
    model = genai.GenerativeModel('gemini-1.5-flash')
    
    # Test command from your previous logic
    user_command = "Pick up the blue box from Zone A and move it to Zone C"
    
    # System Instructions for JSON output
    prompt = f"""
    System: You are an AI Robot Controller. Convert natural language into structured JSON tasks.
    User Command: {user_command}
    Output Format: 
    {{
        "task": "string",
        "target": "string",
        "source": "string",
        "destination": "string",
        "priority": "high/medium/low"
    }}
    """

    try:
        # 3. Execution Logic
        print("--- Robo-Ops AI Orchestrator ---")
        print(f"Input Command: {user_command}")
        
        # Generate Content using Gemini AI
        response = model.generate_content(prompt)
        
        print("\nAI Generated Robot Task:")
        print(response.text)
        
    except Exception as e:
        print(f"Error encountered: {str(e)}")

if __name__ == "__main__":
    robo_ops_orchestrator()
