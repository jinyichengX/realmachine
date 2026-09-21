"use strict";
class ImageViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    this.img = $('<img></img>')
      .css({"width": "100%"})
      .appendTo(this.viewerNode);

    let that = this;

    var mouseOn = false;
    var startX, startY = 0;
    var top, bottom, left, right = 0;

    this.img[0].addEventListener('pointermove', function(e) {
      if(!that.lastMsg) return;
      if(!that.img[0].clientWidth || !that.img[0].clientHeight) return;

      let width = that.img[0].naturalWidth;
      let height = that.img[0].naturalHeight;
      if(that.lastMsg._data_shape) {
        height = that.lastMsg._data_shape[0];
        width = that.lastMsg._data_shape[1];
      }
      let x = e.offsetX / that.img[0].clientWidth * width;
      let y = e.offsetY / that.img[0].clientHeight * height;
      x = Math.min(Math.max(x, 0), width);
      y = Math.min(Math.max(y, 0), height);
      that.tip("(" + x.toFixed(0) + ", " + y.toFixed(0) + ")");
      
      if(mouseOn) {
        var rec_width = Math.abs(e.offsetX - left)
        var rec_height = Math.abs(e.offsetY - top)
        if (x > startX && y > startY) that.drawbox(left, top + 47, rec_width, rec_height, 1);
        else if (x > startX && y < startY) that.drawbox(left, bottom, rec_width, rec_height, 2);
        else if (x < startX && y > startY) that.drawbox(right, top + 47, rec_width, rec_height, 3);
        else if (x < startX && y < startY) that.drawbox(right, bottom, rec_width, rec_height, 4);
        else return;
      }
      
    });
    
    this.img[0].addEventListener('pointerdown', function(e) {
      if(!that.lastMsg) return;
      if(!that.img[0].clientWidth || !that.img[0].clientHeight) return;
      
      let width = that.img[0].naturalWidth;
      let height = that.img[0].naturalHeight;
      if(that.lastMsg._data_shape) {
        height = that.lastMsg._data_shape[0];
        width = that.lastMsg._data_shape[1];
      }
      let x = e.offsetX / that.img[0].clientWidth * width;
      let y = e.offsetY / that.img[0].clientHeight * height;
      console.log("clicked at " + x + ", " + y);
      
      mouseOn = !mouseOn;
      // ros msg
      if(mouseOn) {
        startX = x;
        startY = y;
        left = e.offsetX;
        right = this.width - e.offsetX;
        top = e.offsetY;
        bottom = this.height - e.offsetY;
      }
      else {
        that.drawbox(0,0,0,0,0);
        var ros = new ROSLIB.Ros({
          url : 'ws://192.168.2.67:9090'
        })
        var xyTopic = new ROSLIB.Topic({
          ros: ros,
          name: '/mouse_pos',
          messageType: 'geometry_msgs/Polygon'
        });
        var polygon = new ROSLIB.Message({
          points:[{
            x : startX,
            y : startY,
            z : 0.0
          },
          {
            x : x,
            y : y,
            z : 0.0
          }]
        });
        xyTopic.publish(polygon);
      }
    });

    document.addEventListener('keydown', function(e) {
      if(!that.lastMsg) return;
      if(!that.img[0].clientWidth || !that.img[0].clientHeight) return;
      
      var char = e.isComposing || e.key;
      if (char == ' ') that.status("Tracking");
      else if (char == 'i' || char == 'I') that.status("Identify");
      else if (char == 'r' || char == 'R') that.status("Reset");
      else if (char == 'q' || char == 'Q') that.status("Cancle");
      
      var ros = new ROSLIB.Ros({
        url : 'ws://192.168.2.67:9090'
      })
      var keyTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/key_value',
        messageType: 'std_msgs/Char'
      });
      var key_value = new ROSLIB.Message({
        data: char.charCodeAt()
      });
      keyTopic.publish(key_value);
      
      console.log("key: ", char, " ", char.charCodeAt());
    });
    
    this.lastMsg = null;
    super.onCreate();
  }


  onData(msg) {
    this.card.title.text(msg._topic_name);

    if(msg.__comp) {
      this.decodeAndRenderCompressed(msg);
    } else {
      this.decodeAndRenderUncompressed(msg);
    }
  }
  
  decodeAndRenderCompressed(msg) {
    this.img[0].src = "data:image/jpeg;base64," + msg._data_jpeg;
    this.lastMsg = msg;
  }

  decodeAndRenderUncompressed(msg) {
    this.error("Support for uncompressed images not yet implemented.");
  }
}

ImageViewer.friendlyName = "Image";

ImageViewer.supportedTypes = [
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
    "nav_msgs/msg/OccupancyGrid",
];

ImageViewer.maxUpdateRate = 24.0;

Viewer.registerViewer(ImageViewer);