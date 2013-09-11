
from geometry_msgs.msg import PoseStamped

class TeacherPlugin():
    def __init__(self):
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


class StringTeacher(TeacherPlugin):
    def __init__(self):
        pass

    def getType(self):
        return "string"
    
    def getData(self, name):
        datastr = input('Please enter a value for ' + name +  ' :')
        print datastr
        return datastr
    
    def getRQTData(self, name):
        return self.le.text()
    
    def getRQTWidget(self, name):
        self.le = QLineEdit()
        self.le.setObjectName(name)
        self.le.setText("")
        return self.le
    
    def vizualizeData(self):
        print "halloWelt"


class PoseInputTeacher(TeacherPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "geometry_msgs/PoseStamped"
    
    def getData(self, name):
        p = PoseStamped()
        xval = float(input('Please enter a x value for ' + name +  ' :'))
        yval = float(input('Please enter a y value for ' + name +  ' :'))
        zval = float(input('Please enter a z value for ' + name +  ' :'))
        p.pose.position.x = xval
        p.pose.position.y = yval
        p.pose.position.z = zval
        return p

    def getRQTData(self, name):
        pass
    
    def getRQTWidget(self, name):
        #TODO: 
        #Create VBox with 3 lineedits for x, y, z as prototype
        pass

class PoseTouchupTeacher(TeacherPlugin):
    current_pose = PoseStamped()
    def __init__(self):
        #TODO: create tf listener
        pass
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
