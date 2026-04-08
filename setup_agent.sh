#!/bin/bash
echo "AgentSync Setup"
echo "---------------"
read -p "Enter your name: " name

cat > .env << EOF
SLACK_BOT_TOKEN=xoxb-10866349482854-10862074750835-YrYzTRuHysjW96r83ZKuI21y
SLACK_CHANNEL_ID=C0ARN08DUDA
AGENT_NAME=$name
EOF

echo "Done! You're set up as: $name"
echo "Test it: python3 slack_agent.py post --message \"$name is ready\""
