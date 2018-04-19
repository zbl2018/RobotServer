<?php
$db_host = 'localhost';
$db_name = 'dbtest';
$db_user = 'root';
$db_pwd = '4817';

//面向对象方式
$mysqli = new mysqli($db_host, $db_user, $db_pwd, $db_name);
//面向对象的昂视屏蔽了连接产生的错误，需要通过函数来判断
if(mysqli_connect_error()){
    echo mysqli_connect_error();
}
//设置编码
$mysqli->set_charset("utf8");//或者 $mysqli->query("set names 'utf8'")
//关闭连接
$mysqli->close();
?>