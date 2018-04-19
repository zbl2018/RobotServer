<?php
session_start();

if(isset($_SESSION['user'])=="")
{
 header("Location: index.php");
}
?>

<!DOCTYPE html>
<html lang="utf-8">
<meta http-equiv="Content-Type" content="text/html; charset=utf-8" />
<head>
<script src="js/selectuser.js"></script>
</head>
<title>家居信息显示主页</title>
<body style="text-align:center;">
选择你要查看家的居信息:
<select name="users" onchange="showUser(this.value)">
<option value="1">空调</option>
<option value="2">烟雾报警器</option>
<option value="3">门禁进出历史</option>
<option value="4">电灯状态</option>
</select>
</form>
<p>
<div align="center" id="txtHint"><b>detail info will be listed here.</b></div>
</p>

</body>
<script src="js/jquery.min.js"></script>
<script src="js/common.js"></script>
<!--背景图片自动更换-->
<script src="js/supersized.3.2.7.min.js"></script>
<script src="js/supersized-init.js"></script>
</html>