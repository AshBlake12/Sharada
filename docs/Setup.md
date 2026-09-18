
```sh
python3 --version           
docker --version           
docker run hello-world     
git --version
df -h /                   
```

```sh
curl -LsSf https://astral.sh/uv/install.sh | sh
source ~/.bashrc
uv --version
```

If `docker run hello-world` needs sudo:

```sh
sudo usermod -aG docker $USER
newgrp docker
```


## System packages

```sh
sudo apt update
sudo apt install -y build-essential git python3-venv python3-pip \
  librsvg2-bin pngquant klayout gtkwave \
  libtinfo6 libffi-dev pkg-config
```

## OSS CAD Suite

```sh
mkdir -p ~/eda && cd ~/eda
# Get the latest linux-x64 tarball URL from:
#   https://github.com/YosysHQ/oss-cad-suite-build/releases/latest
wget <oss-cad-suite-linux-x64-YYYYMMDD.tgz URL>
tar -xzf oss-cad-suite-linux-x64-*.tgz
```

```sh
alias oss='source ~/eda/oss-cad-suite/environment'
```

```sh
oss
yosys -V && verilator --version && sby --version
```


## Project repo

```sh
git clone https://github.com/TinyTapeout/tt-support-tools tt
echo "tt/" >> .gitignore
```


```sh
ls -a   
```

Set tiles in `info.yaml`:

```yaml
tiles: "6x4"
```

## LibreLane and the PDK

```sh
mkdir -p ~/ttsetup
uv venv --python 3.12 ~/ttsetup/venv
source ~/ttsetup/venv/bin/activate
uv pip install -r ~/eda/protocol-emulator-asic/tt/requirements.txt
```

Environment variables in `~/ttsetup/env.sh` so you can re-source them:

```sh
export PDK_ROOT=~/ttsetup/pdk
export PDK=ihp-sg13g2
export LIBRELANE_TAG=3.0.3
```

```sh
source ~/ttsetup/env.sh
uv pip install librelane==$LIBRELANE_TAG
```

`librelane --version`

---

## First harden

Note the `--ihp` flag. Every `tt_tool.py` command needs it on this PDK.

```sh
./tt/tt_tool.py --ihp --create-user-config
./tt/tt_tool.py --ihp --harden
```

```sh
./tt/tt_tool.py --ihp --print-warnings
```

```sh
ls runs/wokwi/final/gds/          # a .gds exists
grep -ri "Design area" runs/wokwi/*/reports/ | head
```

Record the cell count and area in the repo now, in `docs/area-log.md`, with the date and commit hash. 

---

## Look at it

```sh
./tt/tt_tool.py --ihp --open-in-klayout
```

If the GUI fails with `could not connect to display :0`:

```sh
xhost +local:docker
```

Worth doing once today. Seeing 24 tiles of actual layout calibrates your sense of how little space you have better than any cell-count table.

---

## RTL tests

New terminal, LibreLane venv active:

```sh
cd ~/eda/protocol-emulator-asic/test
uv pip install -r requirements.txt
make -B
```

cocotb reports passing tests and produces a `.vcd`. Open it with `gtkwave` (from the OSS CAD Suite shell).

---

## UART out of a pin

Their getting-started advice, and the right first design. Replace the template's logic with a minimal UART transmitter:

- Fixed divisor, 8N1, transmit only
- One output pin, one start button on `ui_in`
- Nothing programmable yet

Then: `make -B` in `test/`, then reharden, then record the new cell count in `docs/area-log.md`.

**End-of-day success:** two rows in the area log. You now know what a trivial design costs, which is the baseline every later estimate in the proposal gets checked against.

---

## Hardcaml 

```sh
sudo apt install -y opam
opam init --bare -y
opam switch create 5.2.0
eval $(opam env)
opam install -y hardcaml hardcaml_waveterm dune
```

write a two-line combinational circuit, emit Verilog, read it. That's the whole week-2 spike — if this takes more than an hour, that's the signal to reconsider the mixed-language plan.

