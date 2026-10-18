"""Headless EGL raster depth visibility. Mesh buffers live on GPU once per job."""
import numpy as np
from core import project

NEAR, FAR = .05, 1000.


def projection(k, width, height):
    # Optical +Z forward/+Y down -> GL clip space. Integer image coordinates
    # denote pixel centers; OpenGL window pixel centers have a +0.5 offset.
    p = np.zeros((4,4))
    p[0,:3] = [2*k[0,0]/width, 2*k[0,1]/width, 2*(k[0,2]+.5)/width-1]
    p[1,:3] = [-2*k[1,0]/height, -2*k[1,1]/height, 1-2*(k[1,2]+.5)/height]
    p[2,2], p[2,3] = (FAR+NEAR)/(FAR-NEAR), -2*FAR*NEAR/(FAR-NEAR)
    p[3,2] = 1
    return p


def metric_depth(raw):
    return NEAR*FAR/(FAR-raw*(FAR-NEAR))


def paint_depth(vertices, image, pose, k, depth, colors, best, tolerance=.02):
    """Depth is optical Z in metres, top-left origin; conservative 3x3 visibility."""
    height,width = image.shape[:2]
    sy,sx = depth.shape[0]/height,depth.shape[1]/width
    assigned=0
    for start in range(0,len(vertices),65536):
        camera,uv,ids=project(vertices[start:start+65536],pose,k,width,height)
        xy=np.floor((uv[ids]+.5)*[sx,sy]).astype(int)
        # Minimum over the pixel footprint rejects depth-edge bleed. Background
        # inf never makes an unobserved vertex eligible.
        nearest=np.full(len(ids),np.inf)
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                xx=np.clip(xy[:,0]+dx,0,depth.shape[1]-1)
                yy=np.clip(xy[:,1]+dy,0,depth.shape[0]-1)
                nearest=np.minimum(nearest,depth[yy,xx])
        distance=np.linalg.norm(camera[ids],axis=1)
        visible=(np.abs(nearest-camera[ids,2])<=tolerance)&(distance<best[start+ids])
        local=ids[visible];target=start+local
        pixels=np.rint(uv[local]).astype(int)
        colors[target]=image[pixels[:,1],pixels[:,0]]
        best[target]=distance[visible]
        assigned+=len(target)
    return assigned


class Rasterizer:
    def __init__(self, device=0):
        self.resources=[]
        try:
            import moderngl
            self.gl=moderngl
            self.ctx=moderngl.create_standalone_context(require=330,backend='egl',device_index=device)
            self.resources.append(self.ctx)
            self.renderer=self.ctx.info.get('GL_RENDERER','')
            self.vendor=self.ctx.info.get('GL_VENDOR','')
            if not self.renderer or any(s in self.renderer.lower() for s in
                                       ('llvmpipe','softpipe','swrast','software','swiftshader')):
                raise RuntimeError('Software renderer detected: '+self.renderer)
            self.program=self.ctx.program(
                vertex_shader="""#version 330
                    uniform mat4 mvp;
                    in vec3 position;
                    void main() { gl_Position = mvp * vec4(position, 1.0); }
                """,
                fragment_shader="""#version 330
                    void main() {}
                """)
            self.resources.append(self.program)
            self.vao=None;self.size=None
            self.ctx.enable(moderngl.DEPTH_TEST)
            self.ctx.disable(moderngl.CULL_FACE)
            self.ctx.depth_func='<'
        except Exception as exc:
            self.close()
            raise RuntimeError('GPU着色のEGL初期化に失敗しました。moderngl/libEGLとDockerの'
                               'NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphicsを確認し、'
                               'コンテナを再作成してください。CPUへは自動切替しません。詳細: '+str(exc)) from exc

    def upload(self,vertices,faces):
        if faces.size==0 or faces.min()<0 or faces.max()>=len(vertices):
            raise ValueError('Invalid mesh indices')
        vbo=self.ctx.buffer(np.ascontiguousarray(vertices,dtype='f4'))
        self.resources.append(vbo)
        ibo=self.ctx.buffer(np.ascontiguousarray(faces,dtype='u4'))
        self.resources.append(ibo)
        self.vao=self.ctx.vertex_array(self.program,[(vbo,'3f','position')],index_buffer=ibo)
        self.resources.append(self.vao)

    def depth(self,pose,k,width,height):
        size=(width*2,height*2)
        if self.size!=size:
            if self.size is not None:
                raise ValueError('RGB dimensions changed')
            if max(size)>self.ctx.info['GL_MAX_TEXTURE_SIZE']:
                raise ValueError('Depth framebuffer exceeds GPU texture size')
            self.texture=self.ctx.depth_texture(size)
            self.resources.append(self.texture)
            self.fbo=self.ctx.framebuffer(depth_attachment=self.texture)
            self.resources.append(self.fbo)
            self.size=size
        # The normalized projection remains unchanged under supersampling.
        mvp=projection(k,width,height)@np.linalg.inv(pose)
        self.program['mvp'].write(np.ascontiguousarray(mvp.T,dtype='f4').tobytes())
        self.fbo.use()
        self.ctx.viewport=(0,0,*size)
        self.fbo.clear(depth=1.)
        self.vao.render(mode=self.gl.TRIANGLES)
        raw=np.frombuffer(self.texture.read(alignment=1),dtype=np.float32).reshape(size[1],size[0])[::-1]
        depth=metric_depth(raw.astype(np.float64))
        depth[raw>=1.]=np.inf
        return depth

    def close(self):
        for resource in reversed(self.resources):
            resource.release()
        self.resources=[]


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Check hardware EGL and depth rendering without a bag')
    parser.add_argument('--device',type=int,default=0)
    args=parser.parse_args()
    renderer=Rasterizer(args.device)
    try:
        renderer.upload(np.array([[-1,-1,2],[1,-1,2],[0,1,2]],float),
                        np.array([[0,1,2]]))
        k=np.array([[50.,0,50],[0,50,50],[0,0,1]])
        depth=renderer.depth(np.eye(4),k,101,101)
        if not abs(depth[100,100]-2.) < .001:
            raise RuntimeError('GPU depth smoke test failed')
        print(f'GPU depth OK: {renderer.vendor} / {renderer.renderer}; center={depth[100,100]:.4f} m')
    finally:
        renderer.close()
