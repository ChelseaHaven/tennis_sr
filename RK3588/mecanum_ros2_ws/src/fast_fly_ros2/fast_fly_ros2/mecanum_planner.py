import numpy as np
import casadi as ca



class MecanumModel():
    """Planar model of a holonomic mecanum base, matched to the Gazebo
    force-based move plugin (cmd_vel interface, per-axis velocity cap).

    State  X = [x, y, vx, vy]   (world frame)
    Input  U = [ax, ay]         (world frame acceleration)
    Exact  double-integrator step:
        p' = p + v*dt + 0.5*a*dt^2
        v' = v + a*dt
    """

    def __init__(self, v_max=0.6, a_max=1.0):
        self._v_max = v_max
        self._a_max = a_max

        self._X_dim = 4
        self._U_dim = 2

        self._X_lb = [-20, -20, -v_max, -v_max]
        self._X_ub = [ 20,  20,  v_max,  v_max]
        self._U_lb = [-a_max, -a_max]
        self._U_ub = [ a_max,  a_max]

    def ddynamics_dt(self):
        x = ca.SX.sym('x', self._X_dim)
        u = ca.SX.sym('u', self._U_dim)
        dt = ca.SX.sym('dt')

        pos = x[0:2]
        vel = x[2:4]
        acc = u[0:2]

        pos_next = pos + vel*dt + 0.5*acc*dt*dt
        vel_next = vel + acc*dt
        return ca.Function('ddynamics_dt', [x, u, dt],
                           [ca.vertcat(pos_next, vel_next)])


class MecanumWayPointOpt():
    """Two-phase time-optimal waypoint planner for the mecanum base,
    mirroring WayPointOpt of time_optimal_planner.py:
      phase 1 (define_opt/solve_opt):    feasibility warm-up
      phase 2 (define_opt_t/solve_opt_t): minimize total time
    """

    def __init__(self, model:MecanumModel, wp_num:int, Ns:list, loop:bool, tol=0.02):
        self._loop = loop

        self._model = model
        self._ddynamics = model.ddynamics_dt()

        self._tol = tol
        self._wp_num = wp_num
        assert(len(Ns)==wp_num)
        self._Ns = Ns
        self._Herizon = 0
        self._N_wp_base = [0]
        for i in range(self._wp_num):
            self._N_wp_base.append(self._N_wp_base[i]+self._Ns[i])
            self._Herizon += self._Ns[i]
        print("Total points: ", self._Herizon)

        self._X_dim = self._ddynamics.size1_in(0)
        self._U_dim = self._ddynamics.size1_in(1)
        self._X_lb = model._X_lb
        self._X_ub = model._X_ub
        self._U_lb = model._U_lb
        self._U_ub = model._U_ub

        self._DTs = ca.SX.sym('DTs', self._wp_num)
        self._Xs = ca.SX.sym('Xs', self._X_dim, self._Herizon)
        self._Us = ca.SX.sym('Us', self._U_dim, self._Herizon)
        self._WPs_p = ca.SX.sym('WPs_p', 2, self._wp_num)
        if self._loop:
            self._X_init = self._Xs[:,-1]
        else:
            self._X_init = ca.SX.sym('X_init', self._X_dim)

        self._cost_U = ca.diag([0.01, 0.01])
        self._cost_WP_p = ca.diag([1, 1])

        self._opt_option = {
            'ipopt.tol': 1e-5,
            'ipopt.max_iter': 1000,
            'ipopt.print_level': 0,
        }
        self._opt_t_option = {
            'verbose': False,
            'ipopt.max_iter': 1000,
            'ipopt.print_level': 0
        }
        self._opt_t_warm_option = {
            'verbose': False,
            'ipopt.max_iter': 1000,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_frac': 1e-6,
            'ipopt.warm_start_bound_push': 1e-6,
            'ipopt.warm_start_mult_bound_push': 1e-6,
            'ipopt.warm_start_slack_bound_frac': 1e-6,
            'ipopt.warm_start_slack_bound_push': 1e-6,
            'ipopt.print_level': 0
        }

        #################################################################
        self._nlp_x_x = []
        self._nlp_lbx_x = []
        self._nlp_ubx_x = []

        self._nlp_x_u = []
        self._nlp_lbx_u = []
        self._nlp_ubx_u = []

        self._nlp_x_t = []
        self._nlp_lbx_t = [1e-3]*self._wp_num
        self._nlp_ubx_t = [0.5]*self._wp_num

        self._nlp_g_dyn = []
        self._nlp_lbg_dyn = []
        self._nlp_ubg_dyn = []

        self._nlp_g_wp_p = []
        self._nlp_lbg_wp_p = []
        self._nlp_ubg_wp_p = []

        if self._loop:
            self._nlp_p_xinit = []
        else:
            self._nlp_p_xinit = [ self._X_init ]
            self._xinit = np.array([0,0,0,0])
            # open-ended plan: come to rest at the last waypoint
            self._nlp_g_dyn += [ self._Xs[2:4, self._Herizon-1] ]
            self._nlp_lbg_dyn += [ 0.0, 0.0 ]
            self._nlp_ubg_dyn += [ 0.0, 0.0 ]
        self._nlp_p_dt = []
        self._nlp_p_wp_p = []

        self._nlp_obj_minco = 0
        self._nlp_obj_time = 0
        self._nlp_obj_wp_p = 0
        self._nlp_obj_dyn = 0

        ###################################################################

        for i in range(self._wp_num):
            self._nlp_x_x += [ self._Xs[:, self._N_wp_base[i]] ]
            self._nlp_lbx_x += self._X_lb
            self._nlp_ubx_x += self._X_ub
            self._nlp_x_u += [ self._Us[:, self._N_wp_base[i]] ]
            self._nlp_lbx_u += self._U_lb
            self._nlp_ubx_u += self._U_ub
            self._nlp_x_t += [ self._DTs[i] ]

            if i==0:
                dd_dyn = self._Xs[:,0]-self._ddynamics( self._X_init, self._Us[:,0], self._DTs[0])
            else:
                dd_dyn = self._Xs[:,self._N_wp_base[i]]-self._ddynamics( self._Xs[:,self._N_wp_base[i]-1], self._Us[:,self._N_wp_base[i]], self._DTs[i])
            self._nlp_g_dyn += [ dd_dyn ]
            self._nlp_obj_dyn += dd_dyn.T@dd_dyn

            self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
            self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]

            self._nlp_g_wp_p += [ (self._Xs[0:2,self._N_wp_base[i+1]-1]-self._WPs_p[:,i]).T@(self._Xs[0:2,self._N_wp_base[i+1]-1]-self._WPs_p[:,i]) ]
            self._nlp_lbg_wp_p += [0]
            self._nlp_ubg_wp_p += [ self._tol*self._tol ]

            self._nlp_p_dt += [ self._DTs[i] ]
            self._nlp_p_wp_p += [ self._WPs_p[:,i] ]

            self._nlp_obj_time += self._DTs[i]*self._Ns[i]
            self._nlp_obj_wp_p += (self._Xs[0:2,self._N_wp_base[i+1]-1]-self._WPs_p[:,i]).T@self._cost_WP_p@(self._Xs[0:2,self._N_wp_base[i+1]-1]-self._WPs_p[:,i])

            for j in range(1, self._Ns[i]):
                self._nlp_x_x += [ self._Xs[:, self._N_wp_base[i]+j] ]
                self._nlp_lbx_x += self._X_lb
                self._nlp_ubx_x += self._X_ub
                self._nlp_x_u += [ self._Us[:, self._N_wp_base[i]+j] ]
                self._nlp_lbx_u += self._U_lb
                self._nlp_ubx_u += self._U_ub

                dd_dyn = self._Xs[:,self._N_wp_base[i]+j]-self._ddynamics( self._Xs[:,self._N_wp_base[i]+j-1], self._Us[:,self._N_wp_base[i]+j], self._DTs[i])
                self._nlp_g_dyn += [ dd_dyn ]
                self._nlp_obj_dyn += dd_dyn.T@dd_dyn
                self._nlp_lbg_dyn += [ -0.0 for _ in range(self._X_dim) ]
                self._nlp_ubg_dyn += [  0.0 for _ in range(self._X_dim) ]

                self._nlp_obj_minco += (self._Us[:,self._N_wp_base[i]+j]).T@self._cost_U@(self._Us[:,self._N_wp_base[i]+j])

    # warm-up solver
    def define_opt(self):
        nlp_dect = {
            'f': 1*self._nlp_obj_dyn + self._nlp_obj_wp_p + 1*self._nlp_obj_minco,
            'x': ca.vertcat(*(self._nlp_x_x+self._nlp_x_u)),
            'p': ca.vertcat(*(self._nlp_p_xinit+self._nlp_p_wp_p+self._nlp_p_dt)),
        }
        self._opt_solver = ca.nlpsol('opt', 'ipopt', nlp_dect, self._opt_option)
        self._xu0 = np.zeros((self._X_dim+self._U_dim)*self._Herizon)

    def solve_opt(self, xinit, wp_p, dts):
        if self._loop:
            p = np.zeros(2*self._wp_num+self._wp_num)
            p[:2*self._wp_num] = wp_p
            p[2*self._wp_num:2*self._wp_num+self._wp_num] = dts
        else:
            self._xinit = xinit
            p = np.zeros(self._X_dim+2*self._wp_num+self._wp_num)
            p[:self._X_dim] = xinit
            p[self._X_dim:self._X_dim+2*self._wp_num] = wp_p
            p[self._X_dim+2*self._wp_num:] = dts
        res = self._opt_solver(
            x0=self._xu0,
            lbx=(self._nlp_lbx_x+self._nlp_lbx_u),
            ubx=(self._nlp_ubx_x+self._nlp_ubx_u),
            p=p
        )
        self._xu0 = res['x'].full().flatten()
        self._dt0 = dts
        self._xut0 = np.zeros((self._X_dim+self._U_dim)*self._Herizon+self._wp_num)
        self._xut0[:(self._X_dim+self._U_dim)*self._Herizon] = self._xu0
        self._xut0[(self._X_dim+self._U_dim)*self._Herizon:] = self._dt0
        return res

    # time-optimal solver
    def define_opt_t(self):
        nlp_dect = {
            'f': self._nlp_obj_time,
            'x': ca.vertcat(*(self._nlp_x_x+self._nlp_x_u+self._nlp_x_t)),
            'p': ca.vertcat(*(self._nlp_p_xinit+self._nlp_p_wp_p)),
            'g': ca.vertcat(*(self._nlp_g_dyn+self._nlp_g_wp_p)),
        }
        self._opt_t_solver = ca.nlpsol('opt_t', 'ipopt', nlp_dect, self._opt_t_option)
        self._opt_t_solver_warm = ca.nlpsol('opt_t', 'ipopt', nlp_dect, self._opt_t_warm_option)
        self._lam_x0 = np.zeros(self._opt_t_solver.size_in(6)[0])
        self._lam_g0 = np.zeros(self._opt_t_solver.size_in(7)[0])

    def solve_opt_t(self, xinit, wp_p, warm=False):
        if self._loop:
            p = wp_p
        else:
            p = np.zeros(self._X_dim+2*self._wp_num)
            p[:self._X_dim] = xinit
            p[self._X_dim:] = wp_p
        if warm:
            res = self._opt_t_solver_warm(
                x0=self._xut0,
                lam_x0 = self._lam_x0,
                lam_g0 = self._lam_g0,
                lbx=(self._nlp_lbx_x+self._nlp_lbx_u+self._nlp_lbx_t),
                ubx=(self._nlp_ubx_x+self._nlp_ubx_u+self._nlp_ubx_t),
                lbg=(self._nlp_lbg_dyn+self._nlp_lbg_wp_p),
                ubg=(self._nlp_ubg_dyn+self._nlp_ubg_wp_p),
                p=p
            )
        else:
            res = self._opt_t_solver(
                x0=self._xut0,
                lam_x0 = self._lam_x0,
                lam_g0 = self._lam_g0,
                lbx=(self._nlp_lbx_x+self._nlp_lbx_u+self._nlp_lbx_t),
                ubx=(self._nlp_ubx_x+self._nlp_ubx_u+self._nlp_ubx_t),
                lbg=(self._nlp_lbg_dyn+self._nlp_lbg_wp_p),
                ubg=(self._nlp_ubg_dyn+self._nlp_ubg_wp_p),
                p=p
            )
        if not self._opt_t_solver.stats()['success']:
            raise RuntimeError(self._opt_t_solver.stats()['return_status'])
        self._xut0 = res['x'].full().flatten()
        self._lam_x0 = res["lam_x"]
        self._lam_g0 = res["lam_g"]
        return res


from .gates.gates import Gates
def cal_Ns_2d(gates:Gates, l_per_n:float, loop:bool, init_pos=[0,0,0]):
    if loop:
        init_pos = gates._pos[-1]

    Ns = []
    l = np.linalg.norm(np.array(gates._pos[0][:2])-np.array(init_pos[:2]))
    Ns.append(max(2, int(l/l_per_n)))

    for i in range(gates._N-1):
        l = np.linalg.norm(np.array(gates._pos[i][:2])-np.array(gates._pos[i+1][:2]))
        Ns.append(max(2, int(l/l_per_n)))

    return Ns
