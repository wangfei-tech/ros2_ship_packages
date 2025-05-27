#ifndef OVERLAY_UTIL_H_
#define OVERLAY_UTIL_H_

#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreTexture.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreHardwarePixelBuffer.h>

// 根据 Ogre 版本选择包含路径，ROS2中如果使用 Ogre 1.9 应保持不变
#if OGRE_VERSION < ((1 << 16) | (9 << 8) | 0)
  #include <OGRE/OgrePanelOverlayElement.h>
  #include <OGRE/OgreOverlayElement.h>
  #include <OGRE/OgreOverlayContainer.h>
  #include <OGRE/OgreOverlayManager.h>
#else
  #include <OGRE/Overlay/OgrePanelOverlayElement.h>
  #include <OGRE/Overlay/OgreOverlayElement.h>
  #include <OGRE/Overlay/OgreOverlayContainer.h>
  #include <OGRE/Overlay/OgreOverlayManager.h>
#endif

#include <QImage>
#include <QColor>

// C++11 智能指针替换 boost::shared_ptr
#include <memory>

namespace ros_ship_visualization
{
  class OverlayObject;

  class ScopedPixelBuffer
  {
  public:
    ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
    virtual ~ScopedPixelBuffer();
    virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
    virtual QImage getQImage(unsigned int width, unsigned int height);
    virtual QImage getQImage(OverlayObject& overlay);
    virtual QImage getQImage(unsigned int width, unsigned int height, QColor& bg_color);
    virtual QImage getQImage(OverlayObject& overlay, QColor& bg_color);
  protected:
    Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
  private:

  };

  // this is a class for putting overlay object on rviz 3D panel.
  // This class suppose to be instantiated in onInitialize method
  // of rviz_common::Display class in ROS 2.
  class OverlayObject
  {
  public:
    // 使用 std::shared_ptr 替代 boost::shared_ptr
    typedef std::shared_ptr<OverlayObject> Ptr;

    OverlayObject(const std::string& name);
    virtual ~OverlayObject();

    virtual std::string getName();
    virtual void hide();
    virtual void show();
    virtual bool isTextureReady();
    virtual bool updateTextureSize(unsigned int width, unsigned int height);
    virtual ScopedPixelBuffer getBuffer();
    virtual void setPosition(double left, double top);
    virtual void setDimensions(double width, double height);
    virtual bool isVisible();
    virtual unsigned int getTextureWidth();
    virtual unsigned int getTextureHeight();
  protected:
    const std::string name_;
    Ogre::Overlay* overlay_;
    Ogre::PanelOverlayElement* panel_;
    Ogre::MaterialPtr panel_material_;
    Ogre::TexturePtr texture_;

  private:

  };

  // Ogre::Overlay* createOverlay(std::string name);
  // Ogre::PanelOverlayElement* createOverlayPanel(Ogre::Overlay* overlay);
  // Ogre::MaterialPtr createOverlayMaterial(Ogre::Overlay* overlay);
}

#endif  // OVERLAY_UTIL_H_
