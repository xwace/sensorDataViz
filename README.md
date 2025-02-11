proto文件转为cc和h文件：
进入到proto文件所在目录执行命令：
/usr/local/bin/protoc --proto_path=. --cpp_out=. ./sensor_data.proto
/usr/local/bin/protoc是绝对路径下的so执行文件
