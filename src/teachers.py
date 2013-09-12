
from geometry_msgs.msg import PoseStamped

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
    def getData(self, name):
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
        return self.le.text()
    
    def getRQTWidget(self, name):
        self.le = QLineEdit()
        self.le.setObjectName(name)
        self.le.setText("")
        return self.le
    
    def getName(self):
        return "StringInputTeacher"

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

    def getRQTData(self, name):
        pass


class PoseInputTeacher(TeacherPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "geometry_msgs/PoseStamped"
    
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
        wval_ori = float(input('Please enter a orientation z value for ' + name +  ' :'))
        
        p.pose.orientation.x = xval_ori
        p.pose.orientation.y = yval_ori
        p.pose.orientation.z = zval_ori
        p.pose.orientation.w = wval_ori

        return p

    def getRQTData(self, name):
        pass
    
    def getRQTWidget(self, name):
        #TODO: 
        #Create VBox with 3 lineedits for x, y, z as prototype
        pass

    def getName(self):
        return "PoseInputTeacher"

class PoseTouchupTeacher(TeacherPlugin):
    current_pose = PoseStamped()
    def __init__(self):
        #TODO: create tf listener
        pass

    def getName(self):
        return "PoseTouchupTeacher"

    def getType(self):
        return "geometry_msgs/PoseStamped"

    def getData(self, name):
        #TODO: lookup transform /base_link to /arm_ee_link
        p = PoseStamped()
        return p

    def getRQTData(self, name):
        return self.current_pose
        
    def getRQTWidget(self, name):
        #TODO: 
        #Create a trigger button with callback that does tf lookup and updates self.current_pose
        pass
