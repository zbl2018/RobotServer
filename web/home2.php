<?php
session_start();
//include_once 'Dbconnect.php';
if(!isset($_SESSION['user']))
{
 header("Location: index.php");
}
$mysqli = new mysqli('localhost','root', '4817','dbtest');
//面向对象的昂视屏蔽了连接产生的错误，需要通过函数来判断
if(mysqli_connect_error()){
    echo mysqli_connect_error();
}
//设置编码
$mysqli->set_charset("utf8");//或者 $mysqli->query("set names 'utf8'")
echo $_SESSION['user'];
$res=$mysqli->query("SELECT * FROM users WHERE user_id=".$_SESSION['user'])or die("查询失败");
//echo "222";
//echo $res;
$userRow=$res->fetch_array();
echo "1111";
$mysqli->close();
?>

<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN" "http://www.w3.org/TR/xhtml1/DTD/xhtml1-transitional.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">
<head>
<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
<title>Welcome - <?php echo $userRow['email']; ?></title>
<link rel="stylesheet" href="style.css" type="text/css" />
</head>
<body>
<div id="header">
 <div id="left">
    <label>cleartuts</label>
    </div>
    <div id="right">
     <div id="content">
         hi' <?php echo $userRow['username']; ?>
         <a href="logout.php">Sign Out</a>
        </div>
    </div>
</div>
</body>
</html>