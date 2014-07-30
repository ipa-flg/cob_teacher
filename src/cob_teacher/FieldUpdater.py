class UpdaterPlugin():
    def __init__(self):
        raise NotImplementedError( "Should have implemented this" )

    # Returns name of the Updater routine
    def getName(self):
        raise NotImplementedError( "Should have implemented this" )

    # Returns the type this Updater implements
    def getType(self):
        raise NotImplementedError( "Should have implemented this" )

    # Returns updated field
    def update(self, fieldname, updated_field, input_data):
        raise NotImplementedError( "Should have implemented this" )

class PoseStampedUpdater(UpdaterPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "geometry_msgs/PoseStamped"
    
    def getName(self):
        return "PoseStampedUpdater"

    def update(self, fieldname, updated_field, input_data):
        
        updated_field[fieldname]["header"]["frame_id"] = str(input_data.header.frame_id)

        updated_field[fieldname]["pose"]["position"]["x"] = float(input_data.pose.position.x)
        updated_field[fieldname]["pose"]["position"]["y"] = float(input_data.pose.position.y)
        updated_field[fieldname]["pose"]["position"]["z"] = float(input_data.pose.position.z)

        updated_field[fieldname]["pose"]["orientation"]["x"] = float(input_data.pose.orientation.x)
        updated_field[fieldname]["pose"]["orientation"]["y"] = float(input_data.pose.orientation.y)
        updated_field[fieldname]["pose"]["orientation"]["z"] = float(input_data.pose.orientation.z)
        updated_field[fieldname]["pose"]["orientation"]["w"] = float(input_data.pose.orientation.w)

        return updated_field

class StringUpdater(UpdaterPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "string"
    
    def getName(self):
        return "StringUpdater"

    def update(self, fieldname, updated_field, input_data):
        updated_field[fieldname] = str(input_data)
        return updated_field

class StdStringUpdater(UpdaterPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "std_msgs/String"
    
    def getName(self):
        return "StdStringUpdater"

    def update(self, fieldname, updated_field, input_data):
        updated_field[fieldname]["data"] = str(input_data.data)
        return updated_field

class FloatUpdater(UpdaterPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "float"
    
    def getName(self):
        return "FloatUpdater"

    def update(self, fieldname, updated_field, input_data):
        updated_field[fieldname] = float(input_data)
        return updated_field

class IntUpdater(UpdaterPlugin):
    def __init__(self):
        pass
    
    def getType(self):
        return "int"
    
    def getName(self):
        return "IntUpdater"

    def update(self, fieldname, updated_field, input_data):
        updated_field[fieldname] = int(input_data)
        return updated_field