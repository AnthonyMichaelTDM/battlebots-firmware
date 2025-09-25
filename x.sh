workspaces=(
    bins
    drivers
)

for ws in "${workspaces[@]}"; do
    if [ ! -d "$ws" ]; then
        echo "Error: Workspace directory '$ws' does not exist."
        exit 1
    fi
done

help_msg="Usage: $0 {build|test|clean|check|clippy|flash}"

first_arg=$1
rest="${@:2}"

if [ -z "$first_arg" ]; then
    echo "$help_msg"
    exit 1
fi

source ./export-esp.sh

case $first_arg in
    build)
        for ws in "${workspaces[@]}"; do
            echo "Building workspace: $ws"
            cd $ws 
            cargo build $rest
            cd -
        done
        ;;
    test)
        for ws in "${workspaces[@]}"; do
            echo "Testing workspace: $ws"
            cd $ws 
            cargo test $rest
            cd -
        done
        ;;
    clean)
        for ws in "${workspaces[@]}"; do
            echo "Cleaning workspace: $ws"
            cd $ws 
            cargo clean $rest
            cd -
        done
        ;;
    check)
        for ws in "${workspaces[@]}"; do
            echo "Checking workspace: $ws"
            cd $ws 
            cargo check $rest
            cd -
        done
        ;;
    clippy)
        for ws in "${workspaces[@]}"; do
            echo "Running clippy on workspace: $ws"
            cd $ws 
            cargo clippy $rest
            cd -
        done
        ;;
    flash)
        bin=$2
        rest="${@:3}"
        cd bins
        cargo run --bin $bin $rest
        cd -
        ;;
    *)
        echo "$help_msg"
        exit 1
        ;;
esac