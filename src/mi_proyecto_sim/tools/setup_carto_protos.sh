#!/usr/bin/env bash
# Compila TODOS los .proto de Cartographer 2D a Python. Corre esto UNA sola vez.
# Genera los modulos _pb2.py en tools/carto_protos/cartographer/...
#
# Requisitos: git, protobuf-compiler.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${SCRIPT_DIR}/carto_protos"
CARTO_TAG="${CARTO_TAG:-2.0.0}"
CARTO_SRC="${CARTO_SRC:-/tmp/cartographer_proto_src}"

if ! command -v protoc >/dev/null 2>&1; then
  echo "ERROR: protoc no encontrado. Instala con: apt install -y protobuf-compiler" >&2
  exit 1
fi

if [ ! -d "${CARTO_SRC}/.git" ]; then
  echo ">> Clonando Cartographer ${CARTO_TAG} en ${CARTO_SRC}..."
  git clone --depth 1 --branch "${CARTO_TAG}" \
    https://github.com/cartographer-project/cartographer.git "${CARTO_SRC}"
else
  echo ">> Reutilizando ${CARTO_SRC} (ya clonado)."
fi

rm -rf "${OUT_DIR}"
mkdir -p "${OUT_DIR}"
touch "${OUT_DIR}/__init__.py"

# Compilar TODOS los .proto del subarbol cartographer/. Es mas simple y robusto
# que enumerarlos uno por uno (los paths cambian entre versiones).
mapfile -t PROTO_FILES < <(find "${CARTO_SRC}/cartographer" -name "*.proto" | sort)
echo ">> Compilando ${#PROTO_FILES[@]} .proto con protoc..."

protoc --proto_path="${CARTO_SRC}" \
       --python_out="${OUT_DIR}" \
       "${PROTO_FILES[@]}"

# Crear __init__.py en cada subdirectorio generado para que Python los
# considere paquetes importables.
find "${OUT_DIR}/cartographer" -type d -exec touch {}/__init__.py \;

echo ">> OK. Modulos generados en ${OUT_DIR}/cartographer/."
echo ">> Verificacion rapida:"
ls "${OUT_DIR}/cartographer/mapping/proto/" | grep -E "(submap|grid_2d|pose_graph|serialization)" | sed 's/^/   - /'
