class FieldUpdater():
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

class PoseUpdater(FieldUpdater):
    def __init__(self):
        pass
    
    def getType(self):
        return "geometry_msgs/PoseStamped"
    
    def getName(self):
        return "PoseUpdater"

    def update(self, fieldname, updated_field, input_data):
        
        updated_field[fieldname]["pose"]["position"]["x"] = input_data.pose.position.x
        updated_field[fieldname]["pose"]["position"]["y"] = input_data.pose.position.y
        updated_field[fieldname]["pose"]["position"]["z"] = input_data.pose.position.z

        updated_field[fieldname]["pose"]["orientation"]["x"] = input_data.pose.orientation.x
        updated_field[fieldname]["pose"]["orientation"]["y"] = input_data.pose.orientation.y
        updated_field[fieldname]["pose"]["orientation"]["z"] = input_data.pose.orientation.z
        updated_field[fieldname]["pose"]["orientation"]["w"] = input_data.pose.orientation.w

        return updated_field

class StringUpdater(FieldUpdater):
    def __init__(self):
        pass
    
    def getType(self):
        return "string"
    
    def getName(self):
        return "StringUpdater"

    def update(self, fieldname, updated_field, input_data):
        updated_field[fieldname] = input_data
        return updated_field

class FloatUpdater(FieldUpdater):
    def __init__(self):
        pass
    
    def getType(self):
        return "float"
    
    def getName(self):
        return "FloatUpdater"

    def update(self, fieldname, updated_field, input_data):
        updated_field[fieldname] = input_data
        return updated_field