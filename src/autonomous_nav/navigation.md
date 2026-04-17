        self.status_pub = self.create_publisher(String, "/navigation_status", 10)
        self.feedback_pub = self.create_publisher(Pose2D, "/navigation_feedback", 10)
        self.path_pub = self.create_publisher(Path, "/path", 10)
        self.pos_pub = self.create_publisher(Pose2D, "/pos", 10)