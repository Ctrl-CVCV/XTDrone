import sys

p = "/home/dev/XTDrone-single-car/ws_livox/src/Swarm-LIO2/swarm_lio/launch/dual_mid360_distributed.launch"
s = open(p).read()
pairs = [("quad0_lio_pose_guard", "/iris_0/mavros/local_position/pose"),
         ("quad1_lio_pose_guard", "/iris_1/mavros/local_position/pose")]
for name, topic in pairs:
    anchor = 'name="%s"' % name
    i = s.index(anchor)
    j = s.index("</node>", i)
    insert = "\n    <param name=\"baro_z_topic\" value=\"%s\"/>" % topic
    s = s[:j] + insert + s[j:]
open(p, "w").write(s)
print("patched guards with baro_z_topic")
