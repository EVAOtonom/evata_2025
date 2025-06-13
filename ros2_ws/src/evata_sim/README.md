# EVA Otonom Simülasyon
Simülasyonu kurmadan önce bilgisayarınızda veya sanal makinanızda `Ubuntu 22.04` kurulumu yapmış olmalısınız.
Kurulumunu yaptığınız Ubuntu 22.04 işletim sistemine `ROS2 Humble` versiyonunu kurmalı ve ardından kendi workspace'inizi oluşturmalısınız.

Simülasyon kurulumu için aşağıdaki komutları sırasıyla terminalinizde çalıştırmalısınız.

git clone adımında sizden kullanıcı adınız ve şifreniz istenecektir. Doğru şifrenizi kabul etmediği durumda bunun yerine access token oluşturmanız gerekebilir.

```
cd your_ros_ws
cd src
git clone https://github.com/EVAOtonom/evata_sim.git
cd ..
colcon build
sudo apt-get install ros-humble-ros-gz-bridge
```

Simülasyon ortamınız hazır. Şimdi aşağıdaki komutlar ile simülasyonu başlatabilirsiniz.
```
source install/setup.bash
ros2 launch evata_sim pist_launch.py
```
