from robopatrol.DAO.WaypointDao import WaypointDao

waypoint = WaypointDao()
JData = waypoint.get_waypoints()
print(JData)