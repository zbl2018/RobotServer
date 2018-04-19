<?php
session_start();
$q=$_GET["q"];
echo $q;
//sleep(3);
$con = new mysqli('localhost','root', '4817','web');
if (!$con)
 {
 die('Could not connect: ' . mysql_error());
 }
 $con->set_charset("utf8");//或者 $mysqli->query("set names 'utf8'")
//mysql_select_db("ajax_demo", $con);

$sql="SELECT air_condition_id FROM C_user_AirCondition_info WHERE user_id = '".$_SESSION['user']."'";
// echo "111";
echo $row['air_condition_id'];
$res = $con->query($sql);
$row=$res->fetch_array();

$sql="SELECT * FROM air_condition_info WHERE air_condition_id = '".$row['air_condition_id']."'";

$res = $con->query($sql);

echo "<table border='1'>
<tr>
<th>空调id</th>
<th>品牌</th>
<th>状态</th>
<th>温度</th>
<th>风速</th>
<th>风向</th>
<th>模式</th>
<th>开启时间</th>
<th>关机时间</th>
</tr>";

while($row=$res->fetch_array())
 {
 echo "<tr>";
 echo "<td>" . $row['air_condition_id'] . "</td>";
 echo "<td>" . $row['brand'] . "</td>";
 echo "<td>" . $row['status'] . "</td>";
 echo "<td>" . $row['temperature'] . "</td>";
 echo "<td>" . $row['speed'] . "</td>";
 echo "<td>" . $row['orientation'] . "</td>";
 echo "<td>" . $row['mode'] . "</td>";
 echo "<td>" . $row['open_time_floor'] . "</td>";
 echo "<td>" . $row['open_time_floor'] . "</td>";
 //echo "<td>" . $row['Job'] . "</td>";
 echo "</tr>";
 }
echo "</table>";

$con->close();
?>