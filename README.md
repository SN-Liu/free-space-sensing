pycarmaker.rar 为pycarmaker文件，将该文件解压

CM.rar 为CarMaker的TestRun文件，将该文件解压

Read_freespace.py 可获取freespace sensor数据，并输出自车位置，障碍物长度、宽度以及障碍物相对于自车的位置

将pycamaker文件夹与Read_freespace.py setup.py 放入同一文件夹

用以下命令打开carmaker，并选取CM所在文件夹，即可打开所需要的TestRun
C:\IPG\carmaker\win64-11.1.2\bin\CM.exe -cmdport 16660

先运行一次RestRun后结束
然后运行Read_freespace.py，输出数据后，任意时刻重新运行TestRun均可获得自车位置，障碍物长度、宽度以及障碍物相对于自车的位置
Read_freespace.py运行后将会持续输出数据，当不需要其输出数据时需要手动停止程序
