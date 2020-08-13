vim如何调整Linux操作系统socket缓冲区大小
问题:
应用程序通过socket系统调用和远程主机进行通讯，每一个socket都有一个读写缓冲区。读缓冲区保存了远程主机发送过来的数据，如果缓冲区已满，则数据会被丢弃；写缓冲区保存了要发送到远程主机的数据，如果写缓冲区已满，则系统的应用程序在写入数据时会阻塞。大规模Linux环境下，需要优化系统的缓存区大小，以免影响应用程序运行过程的整体性能。
回答:
以root用户登录Linux操作系统。
通过修改配置文件的方式修改操作系统内核参数。

执行如下命令，进入“/etc”文件夹下。
# cd /etc


“/”为Linux的根目录。

建议修改配置文件“sysctl.conf”前先备份。若备份文件名为“sysctl.conf_bak”，则执行如下命令进行备份。
# cp sysctl.conf sysctl.conf_bak

执行如下命令以文本方式打开“sysctl.conf”文件。
# vi sysctl.conf


在“sysctl.conf”文件中修改或者增加（如果不存在则增加）如下内容：
net.core.rmem_default = 8388608
net.core.rmem_max = 16777216
net.core.wmem_default = 20971520
net.core.wmem_max = 16777216
net.core.netdev_max_backlog = 500000
net.core.somaxconn = 65536
net.core.optmem_max = 81920
net.ipv4.tcp_mem = 94500000   915000000   927000000
net.ipv4.tcp_rmem = 4096   87380   16777216
net.ipv4.tcp_wmem = 4096   87380   16777216
net.ipv4.tcp_keepalive_time = 1200
net.ipv4.tcp_keepalive_intvl = 30
net.ipv4.tcp_keepalive_probes = 3
net.ipv4.tcp_sack = 1
net.ipv4.tcp_fack = 1
net.ipv4.tcp_timestamps = 1
net.ipv4.tcp_window_scaling = 1
net.ipv4.tcp_syncookies = 1
net.ipv4.tcp_tw_reuse = 1
#net.ipv4.tcp_tw_recycle = 1
net.ipv4.tcp_fin_timeout = 30
net.ipv4.ip_local_port_range = 1024  65000
net.ipv4.tcp_max_syn_backlog=819200

net.unix.max_dgram_qlen = 30000
执行如下命令使以上参数生效。
# /sbin/sysctl -p
Fixed Ip,You need to do this
sudo route add -net 239.255.255.250 netmask 255.255.255.255 enx000ec6c301db

