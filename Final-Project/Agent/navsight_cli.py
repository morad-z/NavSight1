import google.generativeai as genai
import os
import sys

# 1. Setup
genai.configure(api_key=os.environ["GOOGLE_API_KEY"])
model = genai.GenerativeModel('gemini-1.5-pro-latest') # Use 1.5 Pro for complex reasoning

# 2. Define The Agent Personas (The Prompts we refined)
PROMPT_DEV = """
# Role: Senior Mobile Engine Developer (Navsight)
# Task: Write the implementation for the user's request. 
# Context: Navsight VIO App (GPS-Denied Navigation). Focus on C++/Rust, memory safety, and low latency.
# Output: Just the code block and brief technical comments.
"""

PROMPT_REVIEWER = """
# Role: Code Reviewer (Navsight)
# Task: Audit the provided code for Memory Leaks, Race Conditions, and O(n^2) loops.
# Input Code is provided below.
# Output: 
# - If critical issues: Rewrite the code with fixes.
# - If minor issues: Comment on them.
# - ALWAYS output the final, clean, safe code block at the end.
"""

PROMPT_DOCS = """
# Role: Documentation Specialist (Navsight)
# Task: Create documentation for the provided code.
# Requirements:
# 1. Mermaid.js sequence diagram of the logic.
# 2. Markdown documentation with SI units for any physics/math.
# Input Code is provided below.
"""

def run_orchestrator(task_description):
    print(f"🚀 Starting Navsight Pipeline for: '{task_description}'\n")

    # --- STEP 1: DEVELOPER ---
    print("👨‍💻 Step 1: Developer is working...")
    dev_response = model.generate_content(f"{PROMPT_DEV}\n\nUSER REQUEST: {task_description}")
    raw_code = dev_response.text
    print("   ...Draft complete.")

    # --- STEP 2: REVIEWER ---
    print("🕵️ Step 2: Reviewer is auditing...")
    review_response = model.generate_content(f"{PROMPT_REVIEWER}\n\nINPUT CODE TO REVIEW:\n{raw_code}")
    verified_code = review_response.text
    print("   ...Audit complete.")

    # --- STEP 3: DOC SPECIALIST ---
    print("📚 Step 3: Documenting...")
    doc_response = model.generate_content(f"{PROMPT_DOCS}\n\nFINAL CODE:\n{verified_code}")
    documentation = doc_response.text
    print("   ...Documentation complete.\n")

    # --- FINAL OUTPUT ---
    print("="*40)
    print("NAV_SIGHT FEATURE REPORT")
    print("="*40)
    print(f"\n{verified_code}\n")
    print("-" * 20)
    print(f"\n{documentation}\n")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python navsight_cli.py \"Your feature request here\"")
    else:
        run_orchestrator(sys.argv[1])