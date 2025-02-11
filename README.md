proto文件转为cc和h文件：
进入到proto文件所在目录执行
/usr/local/bin/protoc --proto_path=. --cpp_out=. ./sensor_data.proto
