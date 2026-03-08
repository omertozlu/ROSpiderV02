#!/bin/bash

#If the script is not running try this "chmod +x ./enter_ros.sh" and make it executable

SCRIPT_NAME=${APP_NAME:-$(basename "$0")}

show_help() {
    cat << EOF

Usage: $SCRIPT_NAME [COMMAND] [OPTIONS]

A utility script for enhancing MY workflow on this specific ROS project.

NOTE: for this script to work correctly u should be inside the project folder while launching this script.
NOTE2: u can add this code to your .bashrc or .zshrc (rename the script_name to your liking)
alias script_name='APP_NAME=script_name ~/PATH/TO/SCRIPT'

Commands:
    help            Show this help message.
    run <profile>   Runs a spesific profile of the docker image (defaults to nvidia).
    stop            Stops the running  docker containers.
  

Examples:
   $SCRIPT_NAME run generic   # Runs container with generic profile
   $SCRIPT_NAME run           # Defaults to nvidia and runs container with nvidia profile
   $SCRIPT_NAME stop          # Stops the running docker containers

EOF
}

MODE=$1

case $MODE in
    "run"|"-r"|"--run")
    # Detect Docker Compose Mode
    # Normal Mode: 'docker compose' (V2 Plugin)
    # Legacy Mode: 'docker-compose' (Standalone)
    if docker compose version > /dev/null 2>&1; then
        DOCKER_COMPOSE_CMD="docker compose"
    elif command -v docker-compose > /dev/null 2>&1; then
        DOCKER_COMPOSE_CMD="docker-compose"
    else
        echo "Error: Docker Compose not found."
        exit 1
    fi

    GPU=$2
        case $GPU in

        "nvidia"|"-n"|"--nvidia")
        $DOCKER_COMPOSE_CMD --profile nvidia up -d
        ;;
        "generic"|"-g"|"--generic")
        $DOCKER_COMPOSE_CMD --profile generic up -d
        ;;

        #For my personal ease of use i set the default to nvidia
        *)
        $DOCKER_COMPOSE_CMD --profile nvidia up -d
        ;;
        esac
    

    # Check if the NVIDIA container is running
    if [ "$(docker ps -q -f name=ros_studio_nvidia)" ]; then
        echo "🔌 Connecting to NVIDIA container..."
        docker exec -it ros_studio_nvidia bash
        exit 0
    fi

    # Check if the GENERIC container is running
    if [ "$(docker ps -q -f name=ros_studio_generic)" ]; then
        echo "💻 Connecting to GENERIC container..."
        docker exec -it ros_studio_generic bash
        exit 0
    fi

    echo "Error: No running ROS container found!"
    echo "Try running: docker compose --profile generic up -d"
    exit 1
    ;;


    "stop"|"-s"|"--stop")
    if [[ -z "$(docker ps -q -f name=ros_studio_nvidia)" && -z "$(docker ps -q -f name=ros_studio_generic)" && -z "$(docker ps -q -f name=rospider-base_config)" ]]; then
        echo "❌ No running ROS container found!"
        exit 1
    fi
    echo "Stopping containers..."
    docker stop ros_studio_nvidia ros_studio_generic rospider-base_config-1 2>/dev/null

    ;;


    "help"|"-h"|"--help")
    show_help
    ;;


    ## If no or incorrect commands were given defaults to show_help
    *)
    show_help
    exit 1
    ;;
    
    esac
    