import numpy as np
# import pygame
# from pygame.locals import *
import pyglet
from OpenGL.GL import *
import ctypes
import cv2.aruco as aruco
import cv2
from math import sqrt
import pyrr

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
        color = texture(imageTexture, fragmentTexCoord);
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
            np.array([0.2, 3, 0.2], dtype=np.float32) - \
            np.array([0, 3*box_size/2, 0], dtype=np.float32)

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

class FloorObject:
    def __init__(self, size):
        self._size = size
        self._generate_mesh()
        self._position = np.array([0, 0, 0])


    def _generate_mesh(self):
        vertices = np.array([
            [-self._size, 0, -self._size],
            [self._size, 0, -self._size],
            [self._size, 0, self._size],
            [-self._size, 0, self._size],
        ], dtype=np.float32)

        colors = np.array([
            [0.3, 0.2, 0.2, 1.0],
            [0.3, 0.2, 0.2, 1.0],
            [0.3, 0.2, 0.2, 1.0],
            [0.3, 0.2, 0.2, 1.0]
        ], dtype=np.float32)

        texture_coordinates = np.array([
            [-0.1, 0.0],
            [-0.1, 0.0],
            [-0.1, 0.0],
            [-0.1, 0.0],
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
        glBindVertexArray(self._vao)

        model_matrix = pyrr.matrix44.create_identity(dtype=np.float32)
        model_matrix = pyrr.matrix44.multiply(model_matrix, pyrr.matrix44.create_from_translation(self._position))

        # glUniform1i(glGetUniformLocation(shader, "useTexture"), GL_FALSE)
        glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, model_matrix.flatten())

        glDrawArrays(GL_QUADS, 0, 4)
        glBindVertexArray(0)
        
class Renderer:
    def __init__(self, width, height):
        self._width = width
        self._height = height

        self.camera_position = np.array([1, 1, 0], dtype=np.float32)
        self.camera_eulers = np.array([0, 0, 0], dtype=np.float32)

        self._init_pygame()
        self._init_opengl()

        self._floor = FloorObject(50)
        self._marker_objects = []

    def start_loop(self):
        self._loop()

    def add_marker_object(self, marker_object):
        self._marker_objects.append(marker_object)

    def set_position(self, position):
        self.camera_position = position

    def set_rotation(self, theta):
        self.camera_eulers[2] = np.deg2rad(theta)

    def _init_pygame(self):
        screen = pyglet.canvas.get_display().get_default_screen()
        template = pyglet.gl.Config(double_buffer=True, depth_size=24)
        try:
            config = screen.get_best_config(template)
        except pyglet.window.NoSuchConfigException:
            template = pyglet.gl.Config()
            config = screen.get_best_config(template)

        self._window = pyglet.window.Window(self._width, self._height, config=config)
        self._window.set_location(0, 0)
        self._window.set_caption('OpenGL')
        self._window.set_mouse_visible(True)

    def _init_opengl(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.shader = create_shader(vertex_source, fragment_source)

        self._projection = pyrr.matrix44.create_perspective_projection_matrix(45, self._width/self._height, 0.1, 50.0)
        self._view = self._get_view_matrix()

        glUseProgram(self.shader)
        glUniformMatrix4fv(glGetUniformLocation(self.shader, "projection"), 1, GL_FALSE, self._projection)
        glUniformMatrix4fv(glGetUniformLocation(self.shader, "view"), 1, GL_FALSE, self._view)
        glUseProgram(0)

    def _get_view_matrix(self):
        view = pyrr.matrix44.create_identity()
        view = pyrr.matrix44.multiply(view, pyrr.matrix44.create_from_translation(-self.camera_position, dtype=np.float32))
        view = pyrr.matrix44.multiply(view, pyrr.matrix44.create_from_eulers(self.camera_eulers, dtype=np.float32))
        return view

    def _handle_events(self):
        pass
        # for event in pygame.event.get():
        #     if event.type == pygame.QUIT:
        #         pygame.quit()
        #         quit()
        # keys = pygame.key.get_pressed()
        # if keys[pygame.K_ESCAPE]:
        #     pygame.quit()
        #     quit()

    def _draw(self):
        glClearColor(0.5, 0.6, 0.65, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glUseProgram(self.shader)

        self.camera_eulers += np.radians(np.array([0, 0, 1], dtype=np.float64)) * .1

        view = self._get_view_matrix()

        glUniformMatrix4fv(glGetUniformLocation(self.shader, "view"), 1, GL_FALSE, view)

        self._floor.draw(self.shader)

        for marker_object in self._marker_objects:
            marker_object.draw(self.shader)
            marker_object.set_rotation(marker_object._rotation + np.radians(np.array([0, 1, 0])))

        self._window.flip()

        # pygame.display.flip()

        # self._clock.tick(60)

    def _loop(self):
        while True:
            self._handle_events()
            self._draw()
            # self._clock.tick(60)

    def get_frame(self):
        data = glReadPixels(0, 0, self._width, self._height, GL_BGR, GL_UNSIGNED_BYTE)
        arr = np.frombuffer(data, dtype=np.uint8).reshape(self._height, self._width, 3)
        arr = np.flip(arr, 0)
        arr = np.ascontiguousarray(arr)
        return arr

if __name__ == "__main__":
    renderer = Renderer(1920, 1080)
    m1 = MarkerObject(0, 0.20)
    m1.set_position(np.array([2, 1, -5]))
    m2 = MarkerObject(1, 0.20)
    m2.set_position(np.array([0, 1, -5]))
    # m1.set_rotation(np.array([np.pi/4, 0, np.pi/2]))
    renderer.add_marker_object(m1)
    renderer.add_marker_object(m2)
    renderer.start_loop()