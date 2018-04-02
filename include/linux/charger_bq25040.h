/*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*
*/

#define CHG_EN_SET_N	143
#define CHG_EOC_N		30

typedef enum BQ25040_ChargingModeTag
{
	BQ25040_CM_OFF = 0,
	BQ25040_CM_USB_100,
	BQ25040_CM_USB_500,
	BQ25040_CM_I_SET,
	BQ25040_CM_FACTORY,

	BQ25040_CM_UNKNOWN
}
BQ25040_ChargingMode;

void BQ25040_SetChargingMode(BQ25040_ChargingMode newMode);

