#!/bin/bash
# stretch4_ros2 tests, run against the Docker image built from .github/docker/Dockerfile
# Add new stretch4_ros2 specific tests below using run_test / run_in_ws

# Default values
IMAGE_NAME="${IMAGE_NAME:-stretch4-ros2-test:latest}"
EXPECTED_SHA=""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'
print_info() { echo -e "${YELLOW}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[PASS]${NC} $1"; }
print_failure() { echo -e "${RED}[FAIL]${NC} $1"; }

# Test counters
TESTS_PASSED=0
TESTS_FAILED=0

# Function to run a test
run_test() {
    local test_name="$1"
    local test_command="$2"

    print_info "Running test: $test_name"

    if eval "$test_command"; then
        print_success "$test_name"
        ((TESTS_PASSED++))
        return 0
    else
        print_failure "$test_name"
        ((TESTS_FAILED++))
        return 1
    fi
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -c|--commit)
            EXPECTED_SHA="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -i, --image NAME     Set the image name to test (default: stretch4-ros2-test:latest)"
            echo "  -c, --commit SHA     Expected stretch4_ros2 commit checked out in the workspace"
            echo "  -h, --help           Display this help message"
            exit 0
            ;;
        *)
            print_failure "Unknown option: $1"
            echo "Run '$0 --help' for usage information"
            exit 1
            ;;
    esac
done

if docker info > /dev/null 2>&1; then
    DOCKER_CMD="docker"
else
    DOCKER_CMD="sudo docker"
fi

WS=/home/hello-robot/ament_ws

# Run a command inside the image with ROS and the ament workspace sourced
run_in_ws() {
    $DOCKER_CMD run --rm $IMAGE_NAME bash -c "source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && $1"
}

print_info "Testing stretch4_ros2 in Docker image: $IMAGE_NAME"
echo ""

if [ -n "$EXPECTED_SHA" ]; then
    run_test "Workspace has stretch4_ros2 at $EXPECTED_SHA" \
        "[ \"\$($DOCKER_CMD run --rm $IMAGE_NAME git -C $WS/src/stretch4_ros2 rev-parse HEAD)\" = \"$EXPECTED_SHA\" ]"
fi

run_test "Workspace install/setup.bash exists" \
    "$DOCKER_CMD run --rm $IMAGE_NAME test -f $WS/install/setup.bash"

for pkg in hello_helpers stretch_core stretch_deep_perception stretch_description stretch_kinematics \
           stretch_nav2 stretch_python_bridge stretch_simulation stretch_tag_perception; do
    run_test "Package $pkg is installed" \
        "run_in_ws 'ros2 pkg prefix $pkg' > /dev/null"
done

echo ""
echo "========================================"
echo "Test Summary"
echo "========================================"
echo -e "${GREEN}Passed:${NC} $TESTS_PASSED"
echo -e "${RED}Failed:${NC} $TESTS_FAILED"
echo "Total:  $((TESTS_PASSED + TESTS_FAILED))"
echo "========================================"

if [ $TESTS_FAILED -eq 0 ]; then
    print_success "All tests passed!"
    exit 0
else
    print_failure "Some tests failed!"
    exit 1
fi
