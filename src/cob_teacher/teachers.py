import roslib
import rospy
import tf

from tf.msg import tfMessage
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped

from PyQt4 import QtGui, QtCore

class TeacherPlugin():
    def __init__(self):
        raise NotImplementedError( "Should have implemented this" )

    # Returns name of the teaching routine
    def getName(self):
        raise NotImplementedError( "Should have implemented this" )

    # Returns the type this teacher implements
    def getType(self):
        raise NotImplementedError( "Should have implemented this" )
    
    # The actual teachin routine. Returns data of the type this teacher implements, when run as a terminal application.
    def getData(self, name, current_data):
        raise NotImplementedError( "Should have implemented this" )

    # The actual teachin routine. Returns data of the type this teacher implements, when run as a rqt application.
    def getRQTData(self, name):
        raise NotImplementedError( "Should have implemented this" )

    # Returns the qtwidget that should be shown for an instance of this teacher
    def getRQTWidget(self, name):
        raise NotImplementedError( "Should have implemented this" )

    # Optional way to vizualize data (doen't know if this will be ever used) ;)
    def vizualizeData(self):
        raise NotImplementedError( "Should have implemented this" )


class StringInputTeacher(TeacherPlugin):
    def __init__(self):
        pass

    def getType(self):
        return "string"
    
    def getData(self, name):
        datastr = input('Please enter a value for ' + name +  ' :')
        return datastr
    
    def getRQTData(self, name):
        return self.le_edit.text()
    
    def getRQTWidget(self, name, current_data):
        self.le = QtGui.QWidget()
        group_layout = QtGui.QVBoxLayout()
        self.le.setLayout(group_layout)

        self.le_edit = QtGui.QLineEdit()
        self.le_edit.setObjectName(name)
        self.le_edit.setText(str(current_data))
        group_layout.addWidget(self.le_edit)
        return self.le
    
    def getName(self):
        return 'StringInputTeacher'

    def vizualizeData(self):
        print "halloWelt"

class StdStringInputTeacher(TeacherPlugin):
    def __init__(self):
        pass

    def getType(self):
        return "std_msgs/String"
    
    def getData(self, name):
        newstr = String()
        newstr.data = input('Please enter a value for ' + name +  ' :')
        return newstr
    
    def getRQTData(self, name):
        newstr = String()
        newstr.data =  self.le_edit.text()
        return newstr
    
    def getRQTWidget(self, name, current_data):
        self.le = QtGui.QWidget()
        group_layout = QtGui.QVBoxLayout()
        self.le.setLayout(group_layout)

        self.le_edit = QtGui.QLineEdit()
        self.le_edit.setObjectName(name)
        self.le_edit.setText(str(current_data["data"]))
        group_layout.addWidget(self.le_edit)
        return self.le
    
    def getName(self):
        return 'StdStringInputTeacher'

    def vizualizeData(self):
        print "halloWelt"


class FloatInputTeacher(TeacherPlugin):
    def __init__(self):
        pass

    def getName(self):
        return "FloatInputTeacher"

    def getType(self):
        return "float"

    def getData(self, name):
        data_float = float(input("Please enter a value for " + name + " :"))
        return data_float

    def getRQTWidget(self, name, current_data):
        self.le = QtGui.QWidget()
        group_layout = QtGui.QVBoxLayout()
        self.le.setLayout(group_layout)

        self.le_edit = QtGui.QLineEdit()
        self.le_edit.setObjectName(name)
        self.le_edit.setText(str(current_data))
        group_layout.addWidget(self.le_edit)
        return self.le

    def getRQTData(self, name):
        return self.le_edit.text()

class PoseInputTeacher(TeacherPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "geometry_msgs/PoseStamped"

    def getName(self):
        return "PoseInputTeacher"
    
    def getData(self, name):
        p = PoseStamped()   
        
        xval_pos = float(input('Please enter a position x value for ' + name +  ' :'))
        yval_pos = float(input('Please enter a position y value for ' + name +  ' :'))
        zval_pos = float(input('Please enter a position z value for ' + name +  ' :'))
        
        p.pose.position.x = xval_pos
        p.pose.position.y = yval_pos
        p.pose.position.z = zval_pos
        
        print " "
        
        xval_ori = float(input('Please enter a orientation x value for ' + name +  ' :'))
        yval_ori = float(input('Please enter a orientation y value for ' + name +  ' :'))
        zval_ori = float(input('Please enter a orientation z value for ' + name +  ' :'))
        wval_ori = float(input('Please enter a orientation w value for ' + name +  ' :'))
        
        p.pose.orientation.x = xval_ori
        p.pose.orientation.y = yval_ori
        p.pose.orientation.z = zval_ori
        p.pose.orientation.w = wval_ori

        return p

    def getRQTData(self, name):
        p = PoseStamped()
        p.header.frame_id = self.le_edit_frame_id.text()
        p.pose.position.x = float(self.le_editx.text())
        p.pose.position.y = float(self.le_edity.text())
        p.pose.position.z = float(self.le_editz.text())

        [x,y,z,w] = tf.transformations.quaternion_from_euler(float(self.le_editroll.text()), float(self.le_editpitch.text()), float(self.le_edityaw.text()))

        p.pose.orientation.x = x
        p.pose.orientation.y = y
        p.pose.orientation.z = z
        p.pose.orientation.w = w

        return p
    
    def getRQTWidget(self, name, current_data):
        self.le = QtGui.QWidget()
        grid_layout = QtGui.QGridLayout()
        self.le.setLayout(grid_layout)

        self.le_label_frame_id = QtGui.QLabel("frame_id:")
        self.le_edit_frame_id = QtGui.QLineEdit()
        self.le_edit_frame_id.setObjectName(name)
        #self.le_edit_frame_id.setReadOnly(True)
        self.le_edit_frame_id.setText(str(current_data['header']['frame_id']))
        grid_layout.addWidget(self.le_label_frame_id, 0,0)
        grid_layout.addWidget(self.le_edit_frame_id, 0,1)

        self.le_labelx = QtGui.QLabel("Position X:")
        self.le_editx = QtGui.QLineEdit()
        self.le_editx.setObjectName(name)
        #self.le_editx.setReadOnly(True)
        self.le_editx.setText(str(current_data['pose']['position']['x']))
        grid_layout.addWidget(self.le_labelx, 1,0)
        grid_layout.addWidget(self.le_editx, 1,1)
        
        self.le_labely = QtGui.QLabel("Position Y:")
        self.le_edity = QtGui.QLineEdit()
        self.le_edity.setObjectName(name)
        #self.le_edity.setReadOnly(True)
        self.le_edity.setText(str(current_data['pose']['position']['y']))
        grid_layout.addWidget(self.le_labely, 2,0)
        grid_layout.addWidget(self.le_edity, 2,1)

        self.le_labelz = QtGui.QLabel("Position Z:")
        self.le_editz = QtGui.QLineEdit()
        self.le_editz.setObjectName(name)
        #self.le_editz.setReadOnly(True)
        self.le_editz.setText(str(current_data['pose']['position']['z']))
        grid_layout.addWidget(self.le_labelz, 3,0)
        grid_layout.addWidget(self.le_editz, 3,1)


        [r,p,y] = tf.transformations.euler_from_quaternion(current_data['pose']['orientation'].values())

        self.le_labelroll = QtGui.QLabel("Orientation R:")
        self.le_editroll = QtGui.QLineEdit()
        self.le_editroll.setObjectName(name)
        #self.le_editroll.setReadOnly(True)
        self.le_editroll.setText(str(r))
        grid_layout.addWidget(self.le_labelroll, 1,2)
        grid_layout.addWidget(self.le_editroll, 1,3)

        self.le_labelpitch = QtGui.QLabel("Orientation P:")
        self.le_editpitch = QtGui.QLineEdit()
        self.le_editpitch.setObjectName(name)
        self.le_editpitch.setText(str(p))
        #self.le_editpitch.setReadOnly(True)
        grid_layout.addWidget(self.le_labelpitch, 2,2)
        grid_layout.addWidget(self.le_editpitch, 2,3)

        self.le_labelyaw = QtGui.QLabel("Orientation Y:")
        self.le_edityaw = QtGui.QLineEdit()
        self.le_edityaw.setObjectName(name)
        self.le_edityaw.setText(str(y))
        #self.le_edityaw.setReadOnly(True)
        grid_layout.addWidget(self.le_labelyaw, 3,2)
        grid_layout.addWidget(self.le_edityaw, 3,3)

        return self.le


class PoseTouchupTeacher(TeacherPlugin):
    current_pose = PoseStamped()
    def __init__(self):
        pass

    def getName(self):
        return "PoseTouchupTeacher"

    def getType(self):
        return "geometry_msgs/PoseStamped"

    def getData(self, name):
        return self.current_pose

    def getRQTData(self, name):
        return self.current_pose
        
    def getRQTWidget(self, name, current_data):
        #TODO: 
        #Create a trigger button with callback that does tf lookup and updates self.current_pose
        pass

class PoseTeachInHandleTeacher(TeacherPlugin):
    current_pose = PoseStamped()
    def __init__(self):
        # start listener for pose 
        # (1) listener for message type tfMessage.msg 
        #     self.listener = rospy.Subscriber('tf', tfMessage, self.callback)
        # (2) listener for message type PoseStamped.msg
        self.listener = rospy.Subscriber('MagBot/teach_in_handle_pose', PoseStamped, self.callback)
        pass

    def callback(self, data):
        # (1) in case of message type tfMessage.msg:
        #self.current_pose.pose.position.x = data.transforms[0].transform.translation.x
        #self.current_pose.pose.position.y = data.transforms[0].transform.translation.y
        #self.current_pose.pose.position.z = data.transforms[0].transform.translation.z
        #self.current_pose.pose.orientation.x = data.transforms[0].transform.rotation.x
        #self.current_pose.pose.orientation.y = data.transforms[0].transform.rotation.y
        #self.current_pose.pose.orientation.z = data.transforms[0].transform.rotation.z
        #self.current_pose.pose.orientation.w = data.transforms[0].transform.rotation.w
        
        # (2) in case of message type PoseStamped.msg:
        self.current_pose = data

    def getName(self):
        return "PoseTeachInHandleTeacher"

    def getType(self):
        return "geometry_msgs/PoseStamped"

    def getData(self, name):
        #any_number = input('Please enter any # to stamp current pose: ')
        raw_input('Press any key to stamp current pose:')
        print self.current_pose 
        return self.current_pose

    def getRQTData(self, name):
        p = PoseStamped()
        p.header.frame_id = self.le_edit_frame_id.text()
        p.pose.position.x = float(self.le_editx.text())
        p.pose.position.y = float(self.le_edity.text())
        p.pose.position.z = float(self.le_editz.text())

        [x,y,z,w] = tf.transformations.quaternion_from_euler(float(self.le_editroll.text()), float(self.le_editpitch.text()), float(self.le_edityaw.text()))

        p.pose.orientation.x = x
        p.pose.orientation.y = y
        p.pose.orientation.z = z
        p.pose.orientation.w = w

        return p
        
    def getRQTWidget(self, name, current_data):
        self.le = QtGui.QWidget()
        grid_layout = QtGui.QGridLayout()
        self.le.setLayout(grid_layout)

        self.le_label_frame_id = QtGui.QLabel("frame_id:")
        self.le_edit_frame_id = QtGui.QLineEdit()
        self.le_edit_frame_id.setObjectName(name)
        self.le_edit_frame_id.setReadOnly(True)
        self.le_edit_frame_id.setText(str(current_data['header']['frame_id']))
        grid_layout.addWidget(self.le_label_frame_id, 0,0)
        grid_layout.addWidget(self.le_edit_frame_id, 0,1)

        self.le_labelx = QtGui.QLabel("Position X:")
        self.le_editx = QtGui.QLineEdit()
        self.le_editx.setObjectName(name)
        self.le_editx.setReadOnly(True)
        self.le_editx.setText(str(current_data['pose']['position']['x']))
        grid_layout.addWidget(self.le_labelx, 1,0)
        grid_layout.addWidget(self.le_editx, 1,1)
        
        self.le_labely = QtGui.QLabel("Position Y:")
        self.le_edity = QtGui.QLineEdit()
        self.le_edity.setObjectName(name)
        self.le_edity.setReadOnly(True)
        self.le_edity.setText(str(current_data['pose']['position']['y']))
        grid_layout.addWidget(self.le_labely, 2,0)
        grid_layout.addWidget(self.le_edity, 2,1)

        self.le_labelz = QtGui.QLabel("Position Z:")
        self.le_editz = QtGui.QLineEdit()
        self.le_editz.setObjectName(name)
        self.le_editz.setReadOnly(True)
        self.le_editz.setText(str(current_data['pose']['position']['z']))
        grid_layout.addWidget(self.le_labelz, 3,0)
        grid_layout.addWidget(self.le_editz, 3,1)


        [r,p,y] = tf.transformations.euler_from_quaternion(current_data['pose']['orientation'].values())

        self.le_labelroll = QtGui.QLabel("Orientation R:")
        self.le_editroll = QtGui.QLineEdit()
        self.le_editroll.setObjectName(name)
        self.le_editroll.setReadOnly(True)
        self.le_editroll.setText(str(r))
        grid_layout.addWidget(self.le_labelroll, 1,2)
        grid_layout.addWidget(self.le_editroll, 1,3)

        self.le_labelpitch = QtGui.QLabel("Orientation P:")
        self.le_editpitch = QtGui.QLineEdit()
        self.le_editpitch.setObjectName(name)
        self.le_editpitch.setText(str(p))
        self.le_editpitch.setReadOnly(True)
        grid_layout.addWidget(self.le_labelpitch, 2,2)
        grid_layout.addWidget(self.le_editpitch, 2,3)

        self.le_labelyaw = QtGui.QLabel("Orientation Y:")
        self.le_edityaw = QtGui.QLineEdit()
        self.le_edityaw.setObjectName(name)
        self.le_edityaw.setText(str(y))
        self.le_edityaw.setReadOnly(True)
        grid_layout.addWidget(self.le_labelyaw, 3,2)
        grid_layout.addWidget(self.le_edityaw, 3,3)

        self.le_teach_button = QtGui.QPushButton("Teach Now")
        grid_layout.addWidget(self.le_teach_button, 4,3)
        self.le_teach_button.connect(self.le_teach_button, QtCore.SIGNAL('clicked()'), self.updateRQTValues)



        return self.le

    def updateRQTValues(self):
        self.le_edit_frame_id.setText(str( self.current_pose.header.frame_id))
        self.le_editx.setText(str( self.current_pose.pose.position.x))
        self.le_edity.setText(str( self.current_pose['pose']['position']['y']))
        self.le_editz.setText(str( self.current_pose['pose']['position']['z']))

        [r,p,y] = tf.transformations.euler_from_quaternion( self.current_pose['pose']['orientation'].values())
        self.le_editroll.setText(str(r))
        self.le_editpitch.setText(str(p))
        self.le_edityaw.setText(str(y))