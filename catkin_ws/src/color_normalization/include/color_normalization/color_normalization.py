class Normalizer(object):
    def __init__(self, config_path):
        pass

    def process_image(self, image):
        """At the momemt, this is just a dummy function. Add some color correction here if you like.

        :param image:
        :return:
        """
        image_normalized = self._normalize_color(image)
        return image_normalized

    def _normalize_color(self, image):
        # TODO: ADD CODE HERE, IF YOU LIKE
        return image



