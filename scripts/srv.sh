export PATH="$HOME/.local/bin:$PATH"
pkill -9 -f run_gr00t_server
sleep 5
cd /home/data/groot/Isaac-GR00T
export CUDA_VISIBLE_DEVICES=1
rm -f /home/data/groot/work/srv.log
nohup uv run python -u gr00t/eval/run_gr00t_server.py --model-path /home/data/groot/checkpoints/GR00T-N1.7-LIBERO/libero_object --embodiment-tag libero_sim --device cuda:0 > /home/data/groot/work/srv.log 2>&1 &
for i in $(seq 1 300); do
  if grep -q listening /home/data/groot/work/srv.log; then echo "READY after ${i}s"; break; fi
  sleep 1
done
tail -3 /home/data/groot/work/srv.log
