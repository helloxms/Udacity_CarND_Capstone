cd ~/opt
mkdir tensorflow
cd tensorflow
git clone https://github.com/tensorflow/models.git
git checkout f7e99c0


#https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md

sudo apt-get install protobuf-compiler python-pil python-lxml python-tk
pip install --user Cython
pip install --user contextlib2
pip install --user jupyter
pip install --user matplotlib

# From tensorflow/models/research/
cd ~/opt/tensorflow/models/research
protoc object_detection/protos/*.proto --python_out=.

export PYTHONPATH=$PYTHONPATH:`pwd`:`pwd`/slim:`pwd`/object_detection
