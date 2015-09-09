#/bin/bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:$PATH

IF=wlan2
IP_FILE=$HOME/codes/snap/$HOSTNAME-ip.txt
CUR_IP=`/sbin/ifconfig $IF | grep "inet addr" | awk -F: '{print $2}' | awk '{print $1}'`
OLD_IP=`cat $IP_FILE`
if [ "$OLD_IP" != "$CUR_IP" ]; then
    echo $CUR_IP > $IP_FILE
    svn ci $IP_FILE -m "automatic IP update"
    echo updated ip old: $OLD_IP new: $CUR_IP
else
    echo ip is still $CUR_IP
fi
    
