"""Portable depth convention tests; opt-in real EGL integration on Notebook."""
import os
import unittest
import numpy as np
from gpu import projection, metric_depth, paint_depth, Rasterizer, NEAR, FAR


class DepthTests(unittest.TestCase):
    def test_software_renderer_rejected_and_context_released(self):
        from unittest.mock import Mock, patch
        from types import SimpleNamespace
        ctx=Mock()
        ctx.info={'GL_VENDOR':'Mesa','GL_RENDERER':'llvmpipe'}
        gl=SimpleNamespace(create_standalone_context=Mock(return_value=ctx))
        with patch.dict('sys.modules',moderngl=gl):
            with self.assertRaisesRegex(RuntimeError,'CPUへは自動切替しません'):
                Rasterizer()
        ctx.release.assert_called_once()

    def test_projection_pixel_centers_and_z(self):
        k=np.array([[50.,2,30],[0,60,20],[0,0,1]])
        p=np.array([.3,.2,2.,1.])
        clip=projection(k,100,80)@p
        ndc=clip[:3]/clip[3]
        # GL bottom-left -> optical image top-left.
        window=np.array([(ndc[0]+1)*50-.5,(1-ndc[1])*40-.5])
        expected=(k@p[:3])[:2]/p[2]
        np.testing.assert_allclose(window,expected)
        self.assertAlmostEqual(float(metric_depth((ndc[2]+1)/2)),2.)
        for z,d in [(NEAR,0.),(FAR,1.)]:
            v=projection(k,100,80)@np.array([0,0,z,1])
            self.assertAlmostEqual((v[2]/v[3]+1)/2,d,places=6)

    def test_depth_occlusion_and_gray_background(self):
        k=np.array([[10.,0,10],[0,10,10],[0,0,1]])
        v=np.array([[0,0,2.],[0,0,3.],[0,0,-2.],[.8,0,2.]])
        depth=np.full((42,42),np.inf)
        depth[19:24,19:24]=2.
        image=np.full((21,21,3),[20,80,200],dtype=np.uint8)
        colors=np.full((4,3),170,dtype=np.uint8);best=np.full(4,np.inf)
        self.assertEqual(paint_depth(v,image,np.eye(4),k,depth,colors,best),1)
        np.testing.assert_array_equal(colors[0],[20,80,200])
        self.assertTrue((colors[1:]==170).all())

    @unittest.skipUnless(os.environ.get('KART_TEST_EGL')=='1','Needs hardware EGL on Notebook')
    def test_real_egl_depth_and_occlusion(self):
        # Front square occludes a smaller rear square. Query interior points to
        # avoid the intentionally conservative raster boundary behaviour.
        vertices=np.array([[-1,-1,2],[1,-1,2],[1,1,2],[-1,1,2],
                           [-.5,-.5,3],[.5,-.5,3],[.5,.5,3],[-.5,.5,3]],float)
        faces=np.array([[0,1,2],[0,2,3],[4,5,6],[4,6,7]])
        renderer=Rasterizer()
        self.addCleanup(renderer.close)
        renderer.upload(vertices,faces)
        k=np.array([[50.,0,50],[0,50,50],[0,0,1]])
        depth=renderer.depth(np.eye(4),k,101,101)
        self.assertAlmostEqual(depth[100,100],2.,places=3)
        query=np.array([[0,0,2.],[0,0,3.]])
        colors=np.full((2,3),170,dtype=np.uint8);best=np.full(2,np.inf)
        self.assertEqual(paint_depth(query,np.full((101,101,3),90,dtype=np.uint8),
                                    np.eye(4),k,depth,colors,best),1)
        np.testing.assert_array_equal(colors[0],[90]*3)
        np.testing.assert_array_equal(colors[1],[170]*3)


if __name__=='__main__': unittest.main()
