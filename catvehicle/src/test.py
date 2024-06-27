import xacro
import rclpy
from gazebo_msgs.srv import SpawnEntity
from random import randint


rclpy.init()
nh = rclpy.create_node("node_service")
client = nh.create_client(SpawnEntity,"/spawn_entity")
for i in range(2):
    doc = xacro.parse(open("/home/akshit/generate_labels/src/generate_labels/urdf/car_4w2.urdf.xacro"))

    namespace = "vehicle" + str(i)
    # num = randint(0,16)
    xacro.process_doc(doc=doc,mappings={'robot_namespace' : namespace,'bit1':hex(2**randint(1,15)),'bit2':hex(0),'bit3':hex(0),'bit4':hex(0),'bit5':hex(0)})


    entity_xml = doc.toxml()
    print(entity_xml)
    # print(entity_xml)




    req = SpawnEntity.Request()
    req.name = 'vehicle' + str(i)

    req.xml = entity_xml
    req.reference_frame = 'world'


    req.initial_pose.position.x = 0.0 + 10.0*i
    req.initial_pose.position.y = 0.0 + 10.0*i
    srv_call = client.call_async(req)
    cnt = 0
    while rclpy.ok():
        if srv_call.done():
            nh.get_logger().info('Spawn status: %s' % srv_call.result().status_message)
            # print(cnt)
            break
        rclpy.spin_once(nh)
        cnt+=1


