#!/usr/bin/env julia

using Images
using Colors
using RobotOS

@rosimport sensor_msgs.msg.Image
rostypegen()

import sensor_msgs.msg

fname(hdr, extension) = @sprintf("%s_%05d_%d.%09d.%s", hdr.frame_id, hdr.seq, hdr.stamp.secs, hdr.stamp.nsecs, extension)

const U16 = FixedPointNumbers.Ufixed16
const ENCODINGS = Dict(
    "bgr8"=>BGR{U8},
    "bgra8"=>BGRA{U8},
    "bgr16"=>BGR{U16},
    "bgra16"=>BGRA{U16},
    "mono8"=>Gray{U8},
    "mono16"=>Gray{U16},
    "rgb8"=>RGB{U8},
    "rgba8"=>RGBA{U8},
    "rgb16"=>RGB{U16},
    "rgba16"=>RGBA{U16})
const PROPERTIES = Dict("spatialorder"=>["x", "y"])

function imageCb(imageMsg::msg.Image, loop_rate, save_directory=".", extension="png")
    @show imageMsg.header
    fname_full = joinpath(save_directory, fname(imageMsg.header, extension))

    @assert imageMsg.is_bigendian == 0

    height = Int(imageMsg.height)
    width = Int(imageMsg.width)
    step = Int(imageMsg.step)

    # get the type
    T = ENCODINGS[imageMsg.encoding]
    # split into (padded) rows
    data = reshape(imageMsg.data, (step, height))
    # truncate any padding
    data = data[1:width*sizeof(T),1:height]
    # fix the type
    data = reinterpret(T, data, (width, height))
    #@assert size(data) == (imageMsg.width, imageMsg.height)

    image = Image(data, PROPERTIES)
    @show T, size(image)

    imwrite(image, fname_full)
    @show fname_full

    rossleep(loop_rate)
    nothing
end

init_node("periodic_image_grabber"; argv=ARGS, anonymous=true)
@printf "initialized node\n"

extension = get_param("~extension", "png")
save_directory = get_param("~save_directory", ".")
rate = Float64(get_param("~rate", 1.0))

loop_rate = Rate(rate)
sub = Subscriber{msg.Image}("image", imageCb, (loop_rate,save_directory,extension), queue_size=1)
@printf "subscribed\n"
spin()
@printf "that's all, folks!\n\n"
