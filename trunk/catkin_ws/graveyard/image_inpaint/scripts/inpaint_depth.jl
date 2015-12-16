#!/usr/bin/env julia

using RobotOS
@rosimport sensor_msgs.msg.Image
rostypegen()
using sensor_msgs.msg

function callback(img::Image, pub_obj::Publisher{Image})
  println(img.header)
  publish(pub_obj, img)
end

function loop(pub_obj)
  loop_rate = Rate(5.0)
  while ! is_shutdown()
    #         npt = Point(rand(), rand(), 0.0)
    #         publish(pub_obj, npt)
    rossleep(loop_rate)
  end
end

function main()
  init_node("inpaint_depth")
  pub = Publisher{Image}("depth_inpainted", queue_size=10)
  sub = Subscriber{Image}("depth", callback, (pub,), queue_size=10)
  loop(pub)
end

if ! isinteractive()
  main()
end
