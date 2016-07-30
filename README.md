# Stereo3dcam

★事前準備  
sudo apt-get update  
sudo apt-get upgrade  
sudo rpi-update  
を実施しておいてください。  
※途中で[Y/n]と聞かれたら Yと答えてください。  
ここから本番です。  
① Raspberry Pi上でターミナルを開く  
② GitHubからソースコードを取得  
git clone https://github.com/RobotTeam2/Stereo3dcam.git  
です。  
③ su権限で必要な環境のインストール  
cd Stereo3dcam  
sudo ./install.sh   
を実行。  
④su権限でエディタで  
./libuvc/include/libuvc/libuvc_internal.h  
の216行目の  
‪#‎define‬ LIBUVC_NUM_TRANSFER_BUFS 1    00
の100を1に変更して保存  
⑤ ビルドの実行  
./build.sh  
を実行  
⑥ カメラアプリの実行  
./camera.mt.sh  
を実行。最初は映像が出るまで時間がかかります。  
