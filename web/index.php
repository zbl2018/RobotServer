<?php
session_start();
//include_once 'Dbconnect.php';
if(isset($_SESSION['user'])!="")
{
 header("Location: home.php");
}
$mysqli = new mysqli('localhost','root', '4817','web');
//面向对象的昂视屏蔽了连接产生的错误，需要通过函数来判断
if(mysqli_connect_error()){
    echo mysqli_connect_error();
}
//设置编码
$mysqli->set_charset("utf8");//或者 $mysqli->query("set names 'utf8'")
if(isset($_POST['btn-login']))
{
//echo $_POST['email'];
 //$email = mysql_real_escape_string($_POST['email']);
 //$upass = mysql_real_escape_string($_POST['pass']);
 $email = $_POST['email'];
 $upass = $_POST['pass'];
 //echo $email;
 $res=$mysqli->query("SELECT * FROM user_info WHERE email='$email'")or die("查询失败");
 $row=$res->fetch_array();
 echo $row['password'];
 //sleep(10);
 if($row['password']==md5($upass))
 {
  $_SESSION['user'] = $row['user_id'];
  sleep(1);
  header("Location: home.php");
 }
 else
 {
  ?>
        <script>alert('password is error or the user dose not exit!');</script>
        <?php
 }
 
}
?>
<!DOCTYPE html>
<html lang="utf-8">
<meta name="viewport" charset="utf-8" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>智能家居管理后台</title>

<link rel="stylesheet" href="css/style.css">

<body>

<div class="login-container">
	<h1>智能家居管理后台</h1>
	
	<div class="connect">
		<p>www.gzhuzbl.club</p>
	</div>
	
	<form  method="post" id="loginForm">
		<div>
			<input type="email" name="email" class="email" placeholder="输入邮箱地址" oncontextmenu="return false" onpaste="return false" />
		</div>
		<div>
			<input type="password" name="pass" class="password" placeholder="密码" oncontextmenu="return false" onpaste="return false" />
		</div>
		<button id="submit" name="btn-login" type="submit">登 陆</button>
	</form>

	<a href="register.php">
		<button type="button"  class="register-tis" >还没有账号？</button>
	</a>

</div>

<script src="js/jquery.min.js"></script>
<script src="js/common.js"></script>
<!--背景图片自动更换-->
<script src="js/supersized.3.2.7.min.js"></script>
<script src="js/supersized-init.js"></script>
<!--表单验证-->
<script src="js/jquery.validate.min.js?var1.14.0"></script>

<div style="text-align:center;margin:50px 0; font:normal 14px/24px 'MicroSoft YaHei';">
<p>适用浏览器：360、FireFox、Chrome、Safari、Opera、傲游、搜狗、世界之窗. 不支持IE8及以下浏览器。</p>
<p>Copyright &copy; 2018<a target="_blank" href="http://gzhuzbl.club/">智能家居</a></p>
</div>
</body>
</html>