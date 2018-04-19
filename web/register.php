<?php
session_start();
if(isset($_SESSION['user'])!="")
{
 header("Location: home.php");
}
//include_once 'Dbconnect.php';
$mysqli = new mysqli('localhost','root', '4817','web');
//面向对象的昂视屏蔽了连接产生的错误，需要通过函数来判断
if(mysqli_connect_error()){
    echo mysqli_connect_error();
}
//设置编码
$mysqli->set_charset("utf8");//或者 $mysqli->query("set names 'utf8'")
//关闭连接
//$mysqli->close();
if(isset($_POST['btn-signup']))
{
//  $uname = mysql_real_escape_string($_POST['uname']);
//  $email = mysql_real_escape_string($_POST['email']);
//  $upass = md5(mysql_real_escape_string($_POST['pass']));
$uname=$_POST['uname'];
$email =$_POST['email'];
$upass=md5($_POST['pass']);
//echo $upass;
 if($mysqli->query("INSERT INTO user_info (username,password,email) VALUES('$uname','$upass','$email')"))
 {
  
		$res=$mysqli->query("SELECT * FROM user_info WHERE email='$email'")or die("查询失败");
		 $row=$res->fetch_array();
		 $user_id=$row['user_id'];
		 ?>
        <script>alert('successfully registered');</script>
		<?php
		$_SESSION["user"]=$user_id;
		header("Location: home.php");
 }
 else
 {
  ?>	
        <script>alert('The user already exists!');</script>
        <?php
 }
}
$mysqli->close();
?>

<!DOCTYPE html>
<html lang="zh-CN">
<meta name="viewport" charset="utf-8" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>智能家居管理后台</title>
<link rel="stylesheet" href="css/style.css">
<body>

<div class="register-container">
	<h1>ShareLink</h1>
	
	<div class="connect">
		<p>Link the world. Share to world.</p>
	</div>
	
	<form action="" method="post" id="registerForm">
		<div>
			<input type="text" name="uname" class="username" placeholder="您的用户名" autocomplete="off"/>
		</div>
		<div>
			<input type="password" name="pass" class="password" placeholder="输入密码" oncontextmenu="return false" onpaste="return false" />
		</div>
		<div>
			<input type="password" name="confirm_password" class="confirm_password" placeholder="再次输入密码" oncontextmenu="return false" onpaste="return false" />
		</div>
		<div>
			<input type="text" name="phone_number" class="phone_number" placeholder="输入手机号码" autocomplete="off" id="number"/>
		</div>
		<div>
			<input type="email" name="email" class="email" placeholder="输入邮箱地址" oncontextmenu="return false" onpaste="return false" />
		</div>

		<button id="submit" name="btn-signup" type="submit">注 册</button>
	</form>
	<a href="index.php">
		<button type="button" class="register-tis">已经有账号？</button>
	</a>

</div>

</body>
<script src="js/jquery.min.js"></script>
<script src="js/common.js"></script>
<!--背景图片自动更换-->
<script src="js/supersized.3.2.7.min.js"></script>
<script src="js/supersized-init.js"></script>
<!--表单验证-->
<script src="js/jquery.validate.min.js?var1.14.0"></script>
</html>