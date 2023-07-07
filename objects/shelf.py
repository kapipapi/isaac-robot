from omni.isaac.core.utils.nucleus import get_assets_root_path, is_file
import carb
import omni


class Shelf():
    def __init__(self):
        self.shelf_usd_path = "shelf.usd"
 
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        
        self.usd_path = assets_root_path + self.shelf_usd_path

        try:
            result = is_file(self.usd_path)
        except:
            result = False

        if result:
            omni.usd.get_context().open
        else:
            carb.log_error(
                f"the usd path {usd_path} could not be opened, please make sure that {args.usd_path} is a valid usd file in {assets_root_path}"
            )