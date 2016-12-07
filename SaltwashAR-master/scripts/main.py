# Copyright (C) 2015 Ross D Milligan
# GNU GENERAL PUBLIC LICENSE Version 3 (full notice can be found at https://github.com/rdmilligan/SaltwashAR)

from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import cv2
from PIL import Image
import numpy as np
from configprovider import ConfigProvider
from robot import *
from webcam import Webcam
from markers import Markers
from features import Features
from constants import *
from objloader import *
import math

class SaltwashAR:
 
    # constants
    INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [-1.0,-1.0,-1.0,-1.0],
                               [ 1.0, 1.0, 1.0, 1.0]])

    def __init__(self, debugOn):
        # initialise config
        self.config_provider = ConfigProvider()

        # initialise robots
        self.rocky_robot = RockyRobot()
        self.sporty_robot = SportyRobot()

        # initialize shapes
        self.arrow = None
        self.batman = None
        self.superman = None

        # initialise webcam
        self.webcam = Webcam()

        # initialise markers
        self.markers = Markers(debugOn)
        print "Debug mode: {}".format(debugOn)
        self.markers_cache = None
        self.cache_counter = 0

        # initialise features
        self.features = Features(self.config_provider)
        
        # initialise texture
        self.texture_background = None

        # initialize face classifier
        self.faceCascade = cv2.CascadeClassifier("cascades/haarcascade_frontalface_default.xml")
        self.faceslist = None

        # initialize hands classifier
        self.okeyCascade = cv2.CascadeClassifier("cascades/haarcascade_okaygesture.xml")
        self.okeylist = None
        self.peaceCascade = cv2.CascadeClassifier("cascades/haarcascade_vickygesture.xml")
        self.peacelist = None

        # initialize filters
        self.prev_tvects = [0.0, 0.0]
        self.prev_rvects = [0.0]

    def _init_gl(self):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(33.7, 1.3, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

        # load shapes
        self.arrow = OBJ('models/arrow/arrowC.obj')
        self.batman = OBJ('models/batman/batman1.obj')
        self.superman = OBJ('models/superman/superman0.obj')
        self.rockR = OBJ('models/rock/rockRR.obj')
        self.rockG = OBJ('models/rock/rockGG.obj')
        self.rockB = OBJ('models/rock/rockBB.obj')
        self.paperR = OBJ('models/paper/paperRR.obj')
        self.paperB = OBJ('models/paper/paperBB.obj')
        self.paperG = OBJ('models/paper/paperGG.obj')
        self.scissorsR = OBJ('models/scissors/scissorsRR.obj')
        self.scissorsG = OBJ('models/scissors/scissorsGG.obj')
        self.scissorsB = OBJ('models/scissors/scissorsBB.obj')

        # load robots frames
        self.rocky_robot.load_frames(self.config_provider.animation)
        self.sporty_robot.load_frames(self.config_provider.animation)

        # start webcam thread
        self.webcam.start()

        # assign texture
        glEnable(GL_TEXTURE_2D)
        self.texture_background = glGenTextures(1)

    def _draw_scene(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # reset robots
        self.rocky_robot.reset()
        self.sporty_robot.reset()

        # get image from webcam
        image = self.webcam.get_current_frame()

        # handle background
        self._handle_background(image.copy())

        # handle markers
        self._handle_markers(image.copy())
       
        # handle features
        self.features.handle(self.rocky_robot, self.sporty_robot, image.copy())

        glutSwapBuffers()

    def _handle_background(self, image):
        
        # let features update background image
        image = self.features.update_background_image(image)

        # recognize faces and draw square
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.faceslist = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        )
        bigface = 0
        iternum = 0
        bigfacedict = {}
        for (x, y, w, h) in self.faceslist:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green
            if (w + h) > bigface:
                bigface = w + h
                self.faceindex = iternum
            bigfacedict[iternum] = w + h
            iternum += 1

        x = sorted(bigfacedict.items(), key=lambda x: x[1], reverse=True)
        y = []
        for a, b in x:
            y.append(a)
        if len(x) > 0:
            self.faceindex = y
        else:
            self.faceindex = None

        # # recognize hands and draw square
        # self.okeylist = self.okeyCascade.detectMultiScale(
        #     gray,
        #     scaleFactor=1.1,
        #     minNeighbors=9,
        #     minSize=(30, 30),
        #     flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        # )
        # for (x, y, w, h) in self.okeylist:
        #     cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)  #Red
        #
        # self.peacelist = self.peaceCascade.detectMultiScale(
        #     gray,
        #     scaleFactor=1.1,
        #     minNeighbors=36,
        #     minSize=(30, 30),
        #     flags=cv2.cv.CV_HAAR_SCALE_IMAGE
        # )
        # for (x, y, w, h) in self.peacelist:
        #     cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)  #Blue

        # convert image to OpenGL texture format
        bg_image = cv2.flip(image, 0)
        bg_image = Image.fromarray(bg_image)     
        ix = bg_image.size[0]
        iy = bg_image.size[1]
        bg_image = bg_image.tobytes('raw', 'BGRX', 0, -1)
 
        # create background texture
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
        
        # draw background
        glBindTexture(GL_TEXTURE_2D, self.texture_background)
        glPushMatrix()
        glTranslatef(0.0,0.0,-20.0)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 1.0); glVertex3f(-8.0, -6.0, 0.0)
        glTexCoord2f(1.0, 1.0); glVertex3f( 8.0, -6.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 8.0,  6.0, 0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-8.0,  6.0, 0.0)
        glEnd( )
        glPopMatrix()

    def _handle_markers(self, image):

        # attempt to detect markers
        markers = []

        try:
            markers = self.markers.detect(image)
        except Exception as ex:
            print(ex)

        # manage markers cache
        if markers:
            # if self.cache_counter < 2:
            #     print "Ko before"
            #     if self.markers_cache is not None:
            #         print "----------------------"
            #         print "Overwritten with cache", markers, self.markers_cache
            #         print "----------------------"
            #         markers = self.markers_cache
            self.markers_cache = markers
            self.cache_counter = 2
        elif self.markers_cache: 
            markers = self.markers_cache
            print "Cache"
            self.cache_counter -= 1
            if self.cache_counter <= 0:
                self.markers_cache = None
            # self.markers_cache = None
        else:
            return

        # examine markers to determine if we are playing rock, paper, scissors
        markersRockPaperScissors = {}
        rockPaperScissors = [0, 0, 0]  # [rock, paper, scissors]
        for i, marker in enumerate(markers):
            marker_name = marker[3]
            if marker_name == M7 or marker_name == M8 or marker_name == M10:
                markersRockPaperScissors[i] = [marker_name, False]
                if marker_name == M7:  # Rock
                    rockPaperScissors[0] += 1
                elif marker_name == M8:  # Paper
                    rockPaperScissors[1] += 1
                else:  # Scissors
                    rockPaperScissors[2] += 1
        # determine results rock, paper, scissors
        if len(markersRockPaperScissors) > 1:
            for pos in markersRockPaperScissors:
                marker_name = markersRockPaperScissors[pos][0]
                if marker_name == M7 and rockPaperScissors[0] == 1 and rockPaperScissors[1] == 0 and rockPaperScissors[2] > 0:  # Rock
                    markersRockPaperScissors[pos][1] = True
                elif marker_name == M8 and rockPaperScissors[0] > 0 and rockPaperScissors[1] == 1 and rockPaperScissors[2] == 0:  # Paper
                    markersRockPaperScissors[pos][1] = True
                elif marker_name == M10 and rockPaperScissors[0] == 0 and rockPaperScissors[1] > 0 and rockPaperScissors[2] == 1:  # Scissors
                    markersRockPaperScissors[pos][1] = True

        print markersRockPaperScissors
        print rockPaperScissors

        for i, marker in enumerate(markers):

            rvecs, tvecs, marker_rotation, marker_name, marker_coords = marker

            # Filter weird changes that happen sometimes after 0 difference (Bug in markers?)
            diff0 = self.prev_tvects[0] - tvecs[0]
            diff1 = self.prev_tvects[1] - tvecs[1]
            diff = diff0*diff0 + diff1*diff1
            print diff, self.prev_tvects[0], tvecs[0], self.prev_tvects[1], tvecs[1]
            # if diff > 1.0:
                # tmp0 = self.prev_tvects[0]
                # tmp1 = self.prev_tvects[1]
                # tmp2 = self.prev_rvects[0]
                # print "Weird change"
                # self.prev_tvects[0] = tvecs[0]
                # self.prev_tvects[1] = tvecs[1]
                # self.prev_rvects[0] = rvecs[2]
                # tvecs[0] = tmp0
                # tvecs[1] = tmp1
                # rvecs[0] = tmp2
            # else:
                # self.prev_tvects[0] = tvecs[0]
                # self.prev_tvects[1] = tvecs[1]
                # self.prev_rvects[0] = rvecs[2]

            # correct rotation and translation to center figures in marker
            rvecs[2] -= (math.pi * marker_rotation / 2.0)
            # print ">>>>>>", rvecs[2], tvecs[0], tvecs[1]
            if marker_rotation == 0:
                tvecs[0] += (0.6) * (math.cos(rvecs[2]) - math.sin(rvecs[2]))
                tvecs[1] += (0.6) * (math.cos(rvecs[2]) + math.sin(rvecs[2]))
            elif marker_rotation == 1:
                tvecs[0] -= (0.6) * (math.cos(rvecs[2]) - math.sin(-rvecs[2]))
                tvecs[1] += (0.6) * (math.cos(rvecs[2]) + math.sin(-rvecs[2]))
            elif marker_rotation == 2:
                tvecs[0] -= (0.6) * (math.cos(rvecs[2]) - math.sin(rvecs[2]))
                tvecs[1] -= (0.6) * (math.cos(rvecs[2]) + math.sin(rvecs[2]))
            else: # 3
                tvecs[0] += (0.6) * (math.cos(rvecs[2]) - math.sin(-rvecs[2]))
                tvecs[1] -= (0.6) * (math.cos(rvecs[2]) + math.sin(-rvecs[2]))

            # print rvecs[2], tvecs[0], tvecs[1]

            # rotate to look at closer face
            if len(self.faceslist) > 0 and marker_name == M0:
                # center of the marker
                m_pos = [0.0, 0.0]
                for pos in marker_coords:
                    m_pos[0] += pos[0]
                    m_pos[1] += pos[1]
                m_pos[0] /= len(marker_coords)
                m_pos[1] /= len(marker_coords)
                # center of the face
                p_pos = [self.faceslist[self.faceindex[0]][0] + self.faceslist[self.faceindex[0]][2]/2.0,
                         self.faceslist[self.faceindex[0]][1] + self.faceslist[self.faceindex[0]][3]/2.0]

                rvecs[2] = math.atan2((p_pos[1] - m_pos[1]), (p_pos[0] - m_pos[0]))

            # build view matrix
            rmtx = cv2.Rodrigues(rvecs)[0]

            view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],tvecs[0]],
                                    [rmtx[1][0],rmtx[1][1],rmtx[1][2],tvecs[1]],
                                    [rmtx[2][0],rmtx[2][1],rmtx[2][2],tvecs[2]],
                                    [0.0       ,0.0       ,0.0       ,1.0    ]])

            view_matrix = view_matrix * self.INVERSE_MATRIX

            view_matrix = np.transpose(view_matrix)

            # load view matrix and draw cube
            glPushMatrix()
            glLoadMatrixd(view_matrix)

            if marker_name == M0:
                glCallList(self.arrow.gl_list)
                # self.rocky_robot.next_frame(marker_rotation, self.features.is_speaking(), self.features.get_emotion())
            elif marker_name == M1:
                # self.sporty_robot.next_frame(marker_rotation, self.features.is_speaking(), self.features.get_emotion())
                glCallList(self.batman.gl_list)
            elif marker_name == M2:
                glCallList(self.superman.gl_list)
            elif marker_name == M3:
                glCallList(self.rockR.gl_list)
            elif marker_name == M4:
                glCallList(self.rockG.gl_list)
            elif marker_name == M5:
                glCallList(self.rockB.gl_list)
            # elif marker_name == M6:
                # glCallList(self.paperR.gl_list)
                # pass
            elif marker_name == M8:  # Paper
                if len(markersRockPaperScissors) > 1:
                    if markersRockPaperScissors[i][1]:
                        glCallList(self.paperG.gl_list)
                    else:
                        glCallList(self.paperR.gl_list)
                else:
                    glCallList(self.paperB.gl_list)
            elif marker_name == M10:  # Scissors
                if len(markersRockPaperScissors) > 1:
                    if markersRockPaperScissors[i][1]:
                        glCallList(self.scissorsG.gl_list)
                    else:
                        glCallList(self.scissorsR.gl_list)
                else:
                    glCallList(self.scissorsB.gl_list)
            elif marker_name == M7:  # Rock
                if len(markersRockPaperScissors) > 1:
                    if markersRockPaperScissors[i][1]:
                        glCallList(self.rockG.gl_list)
                    else:
                        glCallList(self.rockR.gl_list)
                else:
                    glCallList(self.rockB.gl_list)

            glColor3f(1.0, 1.0, 1.0)
            glPopMatrix()

    def main(self):
        # setup and run OpenGL
        glutInit()
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        glutInitWindowSize(640, 480)
        glutInitWindowPosition(100, 100)
        glutCreateWindow('Marker based AR')
        glutDisplayFunc(self._draw_scene)
        glutIdleFunc(self._draw_scene)
        self._init_gl()
        glutMainLoop()

if __name__ == "__main__":
    # run an instance of SaltwashAR
    saltwashAR = SaltwashAR(len(sys.argv) > 1)
    saltwashAR.main()
