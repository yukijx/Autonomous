import numpy as np
from OpenGL.GL import *
from OpenGL.GLUT import *
from threading import Thread, Lock
import ctypes
import cv2.aruco as aruco
import cv2
from math import sqrt
import pyrr
import time
from time import perf_counter_ns

vertex_source = """
#version 330 core

layout (location=0) in vec3 vertexPos;
layout (location=1) in vec4 vertexColor;
layout (location=2) in vec2 vertexTexCoord;

out vec2 fragmentTexCoord;
out vec4 fragmentColorCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
    gl_Position = projection * view * model * vec4(vertexPos, 1.0);
    fragmentTexCoord = vertexTexCoord;
    fragmentColorCoord = vertexColor;
}
"""

fragment_source = """
#version 330 core

in vec2 fragmentTexCoord;
in vec4 fragmentColorCoord;

out vec4 color;

uniform bool useTexture;
uniform sampler2D imageTexture;

void main() {
    if (useTexture && fragmentTexCoord.s > 0.0) { 
        color = fragmentColorCoord * texture(imageTexture, fragmentTexCoord);
    } else {
        color = fragmentColorCoord;
    }
}
"""

def create_shader(vertex_source, fragment_source):
    vertex_shader = glCreateShader(GL_VERTEX_SHADER)
    glShaderSource(vertex_shader, vertex_source)
    glCompileShader(vertex_shader)
    if not glGetShaderiv(vertex_shader, GL_COMPILE_STATUS):
        raise RuntimeError(glGetShaderInfoLog(vertex_shader))

    frag_shader = glCreateShader(GL_FRAGMENT_SHADER)
    glShaderSource(frag_shader, fragment_source)
    glCompileShader(frag_shader)
    if not glGetShaderiv(frag_shader, GL_COMPILE_STATUS):
        raise RuntimeError(glGetShaderInfoLog(frag_shader))

    shader = glCreateProgram()
    glAttachShader(shader, vertex_shader)
    glAttachShader(shader, frag_shader)
    glLinkProgram(shader)
    if not glGetProgramiv(shader, GL_LINK_STATUS):
        raise RuntimeError(glGetProgramInfoLog(shader))
    
    glDeleteShader(vertex_shader)
    glDeleteShader(frag_shader)
    
    return shader

class MarkerTexture:
    def __init__(self, id, texture_size, aruco_dict = aruco.DICT_4X4_50):
        self._id = id
        self._texture_size = texture_size
        self._aruco_dict = aruco.getPredefinedDictionary(aruco_dict)
        self._texture = None
        self._generate_texture()

    def _generate_texture(self):
        marker_image = aruco.generateImageMarker(self._aruco_dict, self._id, int(self._texture_size*3/4))
        marker_image = cv2.copyMakeBorder(marker_image, int(self._texture_size/8), int(self._texture_size/8), int(self._texture_size/8), int(self._texture_size/8), cv2.BORDER_CONSTANT, value=[255, 255, 255])
        marker_image = cv2.cvtColor(marker_image, cv2.COLOR_GRAY2RGBA)
        marker_image = np.flipud(marker_image)
        self._texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self._texture_size, self._texture_size, 0, GL_RGBA, GL_UNSIGNED_BYTE, marker_image)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)

    def bind(self):
        glBindTexture(GL_TEXTURE_2D, self._texture)

    def unbind(self):
        glBindTexture(GL_TEXTURE_2D, 0)

class MarkerMesh:
    def __init__(self, marker_size):

        box_size = marker_size * 4/3

        geometry_vertices = np.array([
            [0, -box_size/2, box_size/sqrt(3)],
            [box_size/2, -box_size/2, -box_size*sqrt(3)/6], 
            [-box_size/2, -box_size/2, -box_size*sqrt(3)/6], 
            [0, box_size/2, box_size/sqrt(3)], 
            [box_size/2, box_size/2, -box_size*sqrt(3)/6], 
            [-box_size/2, box_size/2, -box_size*sqrt(3)/6],
        ], dtype=np.float32)

        stick_vertices = np.array(geometry_vertices, dtype=np.float32) * \
            np.array([0.2, 6, 0.2], dtype=np.float32) - \
            np.array([0, 3*box_size, 0], dtype=np.float32)

        color_vertices = np.array([
            [0.5, 1.0, 1.0, 1.0],
            [1.0, 0.5, 0.5, 1.0],
            [1.0, 0.5, 0.5, 1.0],
            [0.5, 1.0, 0.5, 1.0],
            [0.5, 1.0, 0.5, 1.0],
            [0.5, 0.5, 1.0, 1.0],
        ], dtype=np.float32)

        marker_vertices = np.concatenate((geometry_vertices, color_vertices), axis=1)
        stick_vertices = np.concatenate((stick_vertices, color_vertices), axis=1)

        solid_faces = np.array([
            [0, 1, 2], # bottom face
            [0, 1, 3], # side faces
            [1, 3, 4], # ""
            [1, 2, 4],
            [2, 4, 5],
            [0, 2, 5],
            [0, 3, 5],
            [3, 4, 5]  # top face
        ], dtype=np.uint16).flatten()

        texture_coordinates = np.array([
            [0.0, 0.0], # 0
            [0.0, 0.0], # 1
            [0.0, 0.0], # 2

            [0.0, 0.0], # 0
            [1.0, 0.0], # 1
            [0.0, 1.0], # 3

            [1.0, 0.0], # 1
            [0.0, 1.0], # 3
            [1.0, 1.0], # 4

            [0.0, 0.0], # 1
            [1.0, 0.0], # 2
            [0.0, 1.0], # 4

            [1.0, 0.0], # 2
            [0.0, 1.0], # 4
            [1.0, 1.0], # 5

            [1.0, 0.0], # 0
            [0.0, 0.0], # 2
            [0.0, 1.0], # 5

            [1.0, 0.0], # 0
            [1.0, 1.0], # 3
            [0.0, 1.0], # 5
                
            [0.0, 0.0], # 3
            [0.0, 0.0], # 4
            [0.0, 0.0], # 5
        ], dtype=np.float32)

        marker_vertices_cat = np.array([], dtype=np.float32)
        stick_vertices_cat = np.array([], dtype=np.float32)

        for i,v in enumerate(solid_faces):
            marker_vertex = np.concatenate((marker_vertices[v], texture_coordinates[i]), axis=0)
            marker_vertices_cat = np.concatenate((marker_vertices_cat, marker_vertex), axis=0)
            stick_vertex = np.concatenate((stick_vertices[v], np.array([-0.1, -0.1], dtype=np.float32)), axis=0)
            stick_vertices_cat = np.concatenate((stick_vertices_cat, stick_vertex), axis=0)

        all_vertices = np.concatenate((marker_vertices_cat.flatten(), stick_vertices_cat.flatten()), axis=0).flatten()

        self._vao = glGenVertexArrays(1)
        glBindVertexArray(self._vao)

        n_solid_attrs = 9

        self.count = len(all_vertices) // n_solid_attrs

        self._vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glBufferData(GL_ARRAY_BUFFER, all_vertices, GL_STATIC_DRAW)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, n_solid_attrs*4, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, n_solid_attrs*4, ctypes.c_void_p(12))
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, n_solid_attrs*4, ctypes.c_void_p(28))
        glEnableVertexAttribArray(2)

    def bind(self):
        glBindVertexArray(self._vao)

    def unbind(self):
        glBindVertexArray(0)


class MarkerObject:
    def __init__(self, id, size):
        self._id = id
        self._size = size
        self._texture = MarkerTexture(id, 512, aruco.DICT_4X4_50)
        self._mesh = MarkerMesh(size)
        self._position = np.array([0, 0, 0])
        self._rotation = np.array([0, 0, 0])

    def set_position(self, position):
        self._position = position

    def set_rotation(self, rotation):
        self._rotation = rotation

    def draw(self, shader):
        self._mesh.bind()
        self._texture.bind()

        model_matrix = pyrr.matrix44.create_identity(dtype=np.float32)
        model_matrix = pyrr.matrix44.multiply(model_matrix, pyrr.matrix44.create_from_x_rotation(self._rotation[0]))
        model_matrix = pyrr.matrix44.multiply(model_matrix, pyrr.matrix44.create_from_y_rotation(self._rotation[1]))
        model_matrix = pyrr.matrix44.multiply(model_matrix, pyrr.matrix44.create_from_z_rotation(self._rotation[2]))
        model_matrix = pyrr.matrix44.multiply(model_matrix, pyrr.matrix44.create_from_translation(self._position))

        glUniform1i(glGetUniformLocation(shader, "useTexture"), GL_TRUE)
        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, model_matrix.flatten())
        glDrawArrays(GL_TRIANGLES, 0, self._mesh.count)
        self._mesh.unbind()
        self._texture.unbind()

class FloorTexture:
    def __init__(self, texture_size):
        self._texture_size = texture_size
        self._texture = None
        self._generate_texture()

    def _generate_texture(self):
        # create a random grid texture for the floor
        grid = np.random.randint(0, 255, (self._texture_size, self._texture_size, 3), dtype=np.uint8)
        grid = cv2.GaussianBlur(grid, (5, 5), 0)
        grid = np.flipud(grid)
        self._texture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, self._texture_size, self._texture_size, 0, GL_RGB, GL_UNSIGNED_BYTE, grid)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)

    def bind(self):
        glBindTexture(GL_TEXTURE_2D, self._texture)

    def unbind(self):
        glBindTexture(GL_TEXTURE_2D, 0)


class FloorObject:
    def __init__(self, size):
        self._size = size
        self._generate_mesh()
        self._position = np.array([0, 0, 0])
        self._texture = FloorTexture(512)

    def _generate_mesh(self):
        vertices = np.array([
            [-self._size, 0, -self._size],
            [self._size, 0, -self._size],
            [self._size, 0, self._size],
            [-self._size, 0, self._size],
        ], dtype=np.float32)

        colors = np.array([
            [0.4, 0.2, 0.2, 1.0],
            [0.4, 0.2, 0.2, 1.0],
            [0.4, 0.2, 0.2, 1.0],
            [0.4, 0.2, 0.2, 1.0]
        ], dtype=np.float32)

        texture_coordinates = np.array([
            [0.0, 0.0],
            [5.0, 0.0],
            [5.0, 5.0],
            [0.0, 5.0],
        ], dtype=np.float32)

        vertices = np.concatenate((vertices, colors, texture_coordinates), axis=1).flatten()

        self._vao = glGenVertexArrays(1)
        glBindVertexArray(self._vao)

        n_attrs = 9

        self._vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glBufferData(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, n_attrs*4, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, n_attrs*4, ctypes.c_void_p(12))
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, n_attrs*4, ctypes.c_void_p(28))
        glEnableVertexAttribArray(2)

        glBindVertexArray(0)

    def draw(self, shader):
        self._texture.bind()
        glBindVertexArray(self._vao)

        model_matrix = pyrr.matrix44.create_identity(dtype=np.float32)
        model_matrix = pyrr.matrix44.multiply(model_matrix, pyrr.matrix44.create_from_translation(self._position))

        # glUniform1i(glGetUniformLocation(shader, "useTexture"), GL_FALSE)
        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, model_matrix.flatten())

        glDrawArrays(GL_QUADS, 0, 4)
        glBindVertexArray(0)
        self._texture.unbind()
        
class Renderer:
    def __init__(self, hide_window=False):
        self._window_width = 1920
        self._window_height = 1080
        self._hide_window = hide_window
        self._running = False

        self._rover_position = np.array([1, 1, 0], dtype=np.float32)
        self._rover_eulers = np.array([0, 0, 0], dtype=np.float32)

        self._n_cameras = 0
        self._camera_index = 0
        self._camera_widths = []
        self._camera_heights = []
        self._camera_v_fovs = []
        self._camera_positions = []
        self._camera_eulers = []
        self._rendered_frame_keys = []
        self._rendered_frames = {}

        self._init_window()
        self._init_opengl()

        self._floor = FloorObject(200)
        self._marker_objects = []

        self._render_thread = Thread(target=self._render_loop, name=(
            'render'), args=())
        

    def start(self):
        self._running = True
        self._render_thread.start()

    def stop(self):
        self._running = False

    def add_cameras(self, cameras):
        for camera in cameras:
            key = camera._device
            width = camera._width
            height = camera._height
            v_fov = camera._v_fov
            position = camera._position
            yaw = camera._yaw
            self._add_camera(key, width, height, v_fov, position, yaw)

    def _add_camera(self, key, width, height, v_fov, position, yaw):
        self._camera_widths.append(width)
        self._camera_heights.append(height)
        self._camera_v_fovs.append(v_fov)
        self._camera_positions.append(position)
        self._camera_eulers.append(np.array([0, 0, -np.deg2rad(yaw)], dtype=np.float32))
        self._rendered_frames[key] = None
        self._rendered_frame_keys.append(key)
        self._n_cameras += 1

    def get_frame(self, camera_index):
        if self._rendered_frames[camera_index] is None:
            return None
        return self._rendered_frames[camera_index]

    def add_marker_object(self, marker_object):
        self._marker_objects.append(marker_object)

    def set_rover_position(self, lat, lon, alt):
        self._rover_position = np.array([lon, alt, -lat], dtype=np.float32)

    def set_rover_bearing(self, theta):
        self._rover_eulers[2] = -theta

    def set_camera_matrix(self, intrinsic, distortion):
        self._intrinsic = intrinsic
        self._distortion = distortion

    def _render_loop(self):
        while self._running:
            glutMainLoopEvent()
            self._draw()

    def _init_window(self):
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(self._window_width, self._window_height)
        glutCreateWindow("Camera Render")
        glutDisplayFunc(self._draw)
        glutReshapeFunc(self._reshape_func)
        if self._hide_window:
            glutHideWindow()

    def _reshape_func(self, width, height):
        glutReshapeWindow(self._window_width, self._window_height)

    def _set_window_size(self, width, height):
        if width == self._window_width and height == self._window_height:
            return
        self._window_width = width
        self._window_height = height
        glutReshapeWindow(self._window_width, self._window_height)

    def _init_opengl(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.shader = create_shader(vertex_source, fragment_source)

        glUseProgram(self.shader)
        self._view_location = glGetUniformLocation(self.shader, "view")
        self._projection_location = glGetUniformLocation(self.shader, "projection")
        glUseProgram(0)

    def _get_projection_matrix(self, camera_index=0):
        v_fov = self._camera_v_fovs[camera_index]
        width = self._camera_widths[camera_index]
        height = self._camera_heights[camera_index]
        return pyrr.matrix44.create_perspective_projection_matrix(v_fov, width/height, 0.1, 100.0)


    def _get_view_matrix(self, camera_index=0):
        view = pyrr.matrix44.create_identity()
        view = pyrr.matrix44.multiply(view, pyrr.matrix44.create_from_translation(-self._rover_position, dtype=np.float32))
        view = pyrr.matrix44.multiply(view, pyrr.matrix44.create_from_eulers(self._rover_eulers, dtype=np.float32))
        view = pyrr.matrix44.multiply(view, pyrr.matrix44.create_from_translation(-self._camera_positions[camera_index], dtype=np.float32))
        view = pyrr.matrix44.multiply(view, pyrr.matrix44.create_from_eulers(self._camera_eulers[camera_index], dtype=np.float32))
        return view


    def _draw(self):
        if self._n_cameras == 0:
            return
        
        glClearColor(0.5, 0.6, 0.65, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glUseProgram(self.shader)

        projection = self._get_projection_matrix(self._camera_index)
        glUniformMatrix4fv(self._projection_location, 1, GL_FALSE, projection)
        view = self._get_view_matrix(self._camera_index)
        glUniformMatrix4fv(self._view_location, 1, GL_FALSE, view)

        width = self._camera_widths[self._camera_index]
        height = self._camera_heights[self._camera_index]

        glViewport(0, 0, width, height)

        self._set_window_size(width, height)

        self._floor.draw(self.shader)

        for marker_object in self._marker_objects:
            marker_object.draw(self.shader)


        frame_data = glReadPixels(0, 0, width, height, GL_BGRA, GL_UNSIGNED_BYTE)
        glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0)
        key = self._rendered_frame_keys[self._camera_index]
        self._rendered_frames[key] = self._convert_to_np_array(width, height, frame_data)
        self._camera_index = (self._camera_index + 1) % self._n_cameras

        glutSwapBuffers()
        # time.sleep(.1)
            
    def _convert_to_np_array(self, width, height, data):
        arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 4)
        arr = np.flip(arr, 0)
        arr = np.ascontiguousarray(arr)
        return arr

if __name__ == "__main__":
    renderer = Renderer()
    camera_pos_1 = np.array([.2, 0, 0])
    camera_yaw_1 = 0
    camera_width_1 = 640
    camera_height_1 = 480
    camera_v_fov_1 = 60
    renderer._add_camera(0, camera_width_1, camera_height_1, camera_v_fov_1, camera_pos_1, camera_yaw_1)
    camera_pos_2 = np.array([-.2, 0, 0])
    camera_yaw_2 = 30
    camera_width_2 = 1280
    camera_height_2 = 720
    camera_v_fov_2 = 60
    renderer._add_camera('b', camera_width_2, camera_height_2, camera_v_fov_2, camera_pos_2, camera_yaw_2)
    m1 = MarkerObject(0, 0.20)
    m1.set_position(np.array([1, 1, -5]))
    m2 = MarkerObject(1, 0.20)
    m2.set_position(np.array([-1, 1, -5]))
    renderer.add_marker_object(m1)
    renderer.add_marker_object(m2)
    renderer.start()
    time.sleep(1)
    while renderer._running:
        for m in renderer._marker_objects:
            m.set_rotation(m._rotation + np.array([0, 0.1, 0]))
        frame = renderer.get_frame(0)
        if frame is not None:
            cv2.imshow("frame", frame)
            if cv2.waitKey(33) == ord('q'):
                renderer.stop()