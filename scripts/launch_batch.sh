#!/bin/bash

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
source ../install/setup.bash

# Número de repeticiones
N=${1:-5}
shift  # quitamos N del listado de argumentos

# Cualquier otro parámetro se pasa tal cual al launch
EXTRA_ARGS="$@"

LAUNCH_FILE="riai_launch riai.launch.py"

for ((count=1; count<=N; count++)); do
    echo "=============================="
    echo "Lanzando intento $count de $N..."
    echo "=============================="

    # Lanzar el launch completo en foreground y capturar su PID
    ros2 launch $LAUNCH_FILE $EXTRA_ARGS &
    LAUNCH_PID=$!

    # Esperar a que mission_node termine
    while kill -0 $LAUNCH_PID 2>/dev/null; do
        # Si mission_node no existe, termina el launch completo
        if ! ros2 node list | grep -q "/mission_1"; then
            echo "mission_node ha terminado, matando todo el launch..."
            kill $LAUNCH_PID 2>/dev/null
            wait $LAUNCH_PID 2>/dev/null || true
            break
        fi
        sleep 1
    done

    echo "Intento $count completado."
done

echo "Se completaron $N lanzamientos del launch completo."
